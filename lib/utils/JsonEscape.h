#pragma once

#include <string>
#include <string_view>

namespace Json {
// Escapes JSON string contents, including embedded NUL/control bytes.
inline std::string escape(std::string_view value) {
    constexpr char hex[] = "0123456789abcdef";
    std::string out;
    out.reserve(value.size() + 8);
    for (unsigned char c : value) {
        switch (c) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                if (c < 0x20) {
                    out += "\\u00";
                    out += hex[c >> 4];
                    out += hex[c & 15];
                } else {
                    out += static_cast<char>(c);
                }
        }
    }
    return out;
}
} // namespace Json
