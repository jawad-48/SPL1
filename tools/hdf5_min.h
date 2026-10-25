// hdf5_min.h -- a deliberately tiny HDF5 reader, standard library only.
//
// HDF5 is a binary container format. METR-LA's metr-la.h5 was written by pandas, which always
// produces the same simple shape of file, so we do NOT implement the whole HDF5 standard.
// Supported:
//     superblock version 0 or 1, 8-byte offsets and lengths
//     groups stored with a symbol table (B-tree v1 + local heap)      <- what pandas/PyTables writes
//     object headers version 1, with continuation blocks
//     datasets stored contiguously (or compact) and uncompressed
//     data types: fixed-point integers, IEEE floats, fixed-length strings
// Anything else stops with a message that says exactly what was found, instead of guessing.
//
// Every number in the file is little-endian; we assemble values byte by byte so the code does not
// depend on the machine's own byte order.
#pragma once

#include <cstdint>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace h5 {

typedef unsigned long long u64;

// A dataset found in the file: where its bytes are and how to read element i.
struct Array {
    std::vector<u64> dims;
    int typeClass = -1;             // 0 = integer, 1 = float, 3 = fixed-length string
    int elemSize = 0;               // bytes per element
    bool bigEndian = false;
    bool isSigned = false;
    size_t dataPos = 0;             // absolute position of element 0 in the file
    size_t count = 0;               // number of elements
    const std::vector<unsigned char>* file = nullptr;

    long long asInt(size_t i) const {
        if (typeClass != 0) throw std::runtime_error("dataset is not an integer dataset");
        if (elemSize < 1 || elemSize > 8) throw std::runtime_error("unsupported integer size");
        const unsigned char* p = file->data() + dataPos + i * static_cast<size_t>(elemSize);
        u64 v = 0;
        for (int k = 0; k < elemSize; ++k) {
            int byte = bigEndian ? p[k] : p[elemSize - 1 - k];     // most significant byte first
            v = (v << 8) | static_cast<u64>(byte);
        }
        if (isSigned && elemSize < 8 && (v >> (8 * elemSize - 1)) & 1) v |= ~0ULL << (8 * elemSize);
        return static_cast<long long>(v);
    }

    double asDouble(size_t i) const {
        if (typeClass == 0) return static_cast<double>(asInt(i));
        if (typeClass != 1) throw std::runtime_error("dataset is not a number dataset");
        const unsigned char* p = file->data() + dataPos + i * static_cast<size_t>(elemSize);
        unsigned char b[8];
        for (int k = 0; k < elemSize; ++k) b[k] = bigEndian ? p[elemSize - 1 - k] : p[k];   // to little-endian
        if (elemSize == 8) { double d; std::memcpy(&d, b, 8); return d; }
        if (elemSize == 4) { float f; std::memcpy(&f, b, 4); return static_cast<double>(f); }
        throw std::runtime_error("unsupported float size " + std::to_string(elemSize));
    }

    std::string asString(size_t i) const {
        if (typeClass != 3) throw std::runtime_error("dataset is not a fixed-length string dataset");
        const unsigned char* p = file->data() + dataPos + i * static_cast<size_t>(elemSize);
        std::string s(reinterpret_cast<const char*>(p), static_cast<size_t>(elemSize));
        while (!s.empty() && (s.back() == '\0' || s.back() == ' ')) s.pop_back();
        return s;
    }

    std::string describe() const {
        std::string shape = "[";
        for (size_t i = 0; i < dims.size(); ++i) shape += (i ? " x " : "") + std::to_string(dims[i]);
        shape += "]";
        const char* kind = typeClass == 0 ? (isSigned ? "signed int" : "unsigned int") : typeClass == 1 ? "float" : typeClass == 3 ? "string" : "other";
        return shape + ", " + kind + ", " + std::to_string(elemSize * 8) + "-bit elements";
    }
};

class File {
public:
    explicit File(const std::string& path) {
        std::ifstream in(path.c_str(), std::ios::binary);
        if (!in) throw std::runtime_error("cannot open " + path);
        in.seekg(0, std::ios::end);
        std::streamoff size = in.tellg();
        in.seekg(0, std::ios::beg);
        buf_.resize(static_cast<size_t>(size));
        if (size > 0) in.read(reinterpret_cast<char*>(buf_.data()), size);
        parseSuperblock();
    }

    int superblockVersion() const { return sbVersion_; }
    size_t rootHeader() const { return rootHeader_; }

    // All (name, object-header position) pairs stored in a group.
    std::vector<std::pair<std::string, size_t>> listGroup(size_t groupHeader) const {
        size_t btree = 0, heap = 0;
        bool found = false;
        for (const Message& m : messages(groupHeader)) {
            if (m.type == 0x11) { btree = address(u64At(m.pos)); heap = address(u64At(m.pos + 8)); found = true; }
        }
        if (!found) throw std::runtime_error("object is not a group with a symbol table (newer HDF5 link format?)");
        if (!hasSignature(heap, "HEAP")) throw std::runtime_error("local heap not found");
        size_t heapData = address(u64At(heap + 24));
        std::vector<std::pair<std::string, size_t>> out;
        walkTree(btree, heapData, out, 0);
        return out;
    }

    bool findChild(size_t groupHeader, const std::string& name, size_t& header) const {
        for (const auto& entry : listGroup(groupHeader)) {
            if (entry.first == name) { header = entry.second; return true; }
        }
        return false;
    }

    std::string childNames(size_t groupHeader) const {
        std::string s;
        for (const auto& entry : listGroup(groupHeader)) s += (s.empty() ? "" : ", ") + entry.first;
        return s.empty() ? "(none)" : s;
    }

    Array openArray(size_t header) const {
        Array a;
        a.file = &buf_;
        bool haveSpace = false, haveType = false, haveLayout = false;
        for (const Message& m : messages(header)) {
            if (m.type == 0x0B) throw std::runtime_error("dataset uses a filter pipeline (compression); this reader only supports uncompressed data");
            if (m.type == 0x01) {                                   // dataspace
                unsigned version = u8At(m.pos);
                unsigned rank = u8At(m.pos + 1);
                size_t dimsPos = (version == 1) ? m.pos + 8 : m.pos + 4;
                if (version != 1 && version != 2) throw std::runtime_error("unsupported dataspace version " + std::to_string(version));
                a.count = 1;
                for (unsigned i = 0; i < rank; ++i) { a.dims.push_back(u64At(dimsPos + 8 * i)); a.count *= static_cast<size_t>(a.dims.back()); }
                haveSpace = true;
            } else if (m.type == 0x03) {                            // datatype
                a.typeClass = static_cast<int>(u8At(m.pos) & 0x0F);
                unsigned bits = u8At(m.pos + 1);
                a.bigEndian = (a.typeClass == 0 || a.typeClass == 1) && (bits & 1);
                a.isSigned = (a.typeClass == 0) && (bits & 0x08);
                a.elemSize = static_cast<int>(u32At(m.pos + 4));
                haveType = true;
            } else if (m.type == 0x08) {                            // data layout
                unsigned version = u8At(m.pos);
                if (version != 3) throw std::runtime_error("unsupported data layout version " + std::to_string(version));
                unsigned layoutClass = u8At(m.pos + 1);
                if (layoutClass == 1) {                             // contiguous
                    u64 addr = u64At(m.pos + 2);
                    if (addr == ~0ULL) throw std::runtime_error("dataset has no stored data");
                    a.dataPos = address(addr);
                } else if (layoutClass == 0) {                      // compact: data sits inside the header
                    a.dataPos = m.pos + 4;
                } else {
                    throw std::runtime_error("dataset is chunked; this reader only supports contiguous data");
                }
                haveLayout = true;
            }
        }
        if (!haveSpace || !haveType || !haveLayout) throw std::runtime_error("dataset header is missing dataspace, datatype or layout");
        if (a.dataPos + a.count * static_cast<size_t>(a.elemSize) > buf_.size()) throw std::runtime_error("dataset extends beyond the end of the file");
        return a;
    }

private:
    struct Message { unsigned type; size_t pos; unsigned size; };

    std::vector<unsigned char> buf_;
    size_t base_ = 0;
    size_t rootHeader_ = 0;
    int sbVersion_ = -1;

    // ---- little-endian readers with bounds checks
    void need(size_t pos, size_t n) const {
        if (pos > buf_.size() || n > buf_.size() - pos) throw std::runtime_error("read past end of file (corrupt or unsupported file)");
    }
    unsigned u8At(size_t pos) const { need(pos, 1); return buf_[pos]; }
    unsigned u16At(size_t pos) const { need(pos, 2); return buf_[pos] | (buf_[pos + 1] << 8); }
    unsigned u32At(size_t pos) const { need(pos, 4); return static_cast<unsigned>(u16At(pos)) | (static_cast<unsigned>(u16At(pos + 2)) << 16); }
    u64 u64At(size_t pos) const { return static_cast<u64>(u32At(pos)) | (static_cast<u64>(u32At(pos + 4)) << 32); }

    bool hasSignature(size_t pos, const char* sig) const {
        need(pos, 4);
        return std::memcmp(buf_.data() + pos, sig, 4) == 0;
    }

    size_t address(u64 relative) const {
        if (relative == ~0ULL) throw std::runtime_error("undefined address in file");
        return base_ + static_cast<size_t>(relative);
    }

    void parseSuperblock() {
        static const unsigned char signature[8] = {0x89, 'H', 'D', 'F', '\r', '\n', 0x1a, '\n'};
        if (buf_.size() < 64 || std::memcmp(buf_.data(), signature, 8) != 0) throw std::runtime_error("not an HDF5 file (signature not found at offset 0)");
        sbVersion_ = static_cast<int>(u8At(8));
        if (sbVersion_ != 0 && sbVersion_ != 1)
            throw std::runtime_error("unsupported superblock version " + std::to_string(sbVersion_) + " (this reader supports 0 and 1)");
        if (u8At(13) != 8 || u8At(14) != 8) throw std::runtime_error("only 8-byte offsets and lengths are supported");
        size_t pos = (sbVersion_ == 0) ? 24 : 28;                  // base address
        base_ = static_cast<size_t>(u64At(pos));
        size_t rootEntry = pos + 32;                               // after base, free-space, end-of-file, driver info
        rootHeader_ = address(u64At(rootEntry + 8));
    }

    // Header messages of a version-1 object header, following continuation blocks.
    std::vector<Message> messages(size_t header) const {
        unsigned version = u8At(header);
        if (version != 1) throw std::runtime_error("object header version " + std::to_string(version) + " is not supported (only version 1)");
        std::vector<std::pair<size_t, size_t>> blocks;
        blocks.push_back(std::make_pair(header + 16, static_cast<size_t>(u32At(header + 8))));
        std::vector<Message> out;
        for (size_t b = 0; b < blocks.size() && b < 64; ++b) {
            size_t pos = blocks[b].first, end = pos + blocks[b].second;
            while (pos + 8 <= end) {
                unsigned type = u16At(pos), size = u16At(pos + 2);
                size_t data = pos + 8;
                if (data + size > end) break;
                out.push_back(Message{type, data, size});
                if (type == 0x10) blocks.push_back(std::make_pair(address(u64At(data)), static_cast<size_t>(u64At(data + 8))));
                pos = data + size;
            }
        }
        return out;
    }

    // Version-1 B-tree of a group: level 0 nodes point at symbol-table nodes holding the names.
    void walkTree(size_t node, size_t heapData, std::vector<std::pair<std::string, size_t>>& out, int depth) const {
        if (depth > 16 || !hasSignature(node, "TREE")) throw std::runtime_error("group B-tree not found");
        unsigned level = u8At(node + 5), used = u16At(node + 6);
        for (unsigned i = 0; i < used; ++i) {
            size_t child = address(u64At(node + 32 + 16 * i));
            if (level > 0) { walkTree(child, heapData, out, depth + 1); continue; }
            if (!hasSignature(child, "SNOD")) throw std::runtime_error("symbol table node not found");
            unsigned symbols = u16At(child + 6);
            for (unsigned s = 0; s < symbols; ++s) {
                size_t entry = child + 8 + 40 * s;
                size_t namePos = heapData + static_cast<size_t>(u64At(entry));
                std::string name;
                while (u8At(namePos) != 0) name.push_back(static_cast<char>(u8At(namePos++)));
                out.push_back(std::make_pair(name, address(u64At(entry + 8))));
            }
        }
    }
};

}  // namespace h5
