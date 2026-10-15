// Minimal JSON reader for the scenario files of fdt-emu (objects, arrays, numbers, strings, bools,
// null). No dependency outside the C++ standard library.
#ifndef FDT_MINIJSON_H
#define FDT_MINIJSON_H

#include <cstdlib>
#include <fstream>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace mj
{

struct Value
{
    enum Type
    {
        NUL,
        BOOL,
        NUM,
        STR,
        ARR,
        OBJ
    } type = NUL;

    bool b = false;
    double n = 0;
    std::string s;
    std::vector<Value> a;
    std::map<std::string, Value> o;

    bool Has(const std::string& k) const
    {
        return type == OBJ && o.count(k) && o.at(k).type != NUL;
    }

    const Value& operator[](const std::string& k) const
    {
        static Value nul;
        if (type != OBJ)
        {
            return nul;
        }
        auto it = o.find(k);
        return it == o.end() ? nul : it->second;
    }

    const Value& operator[](size_t i) const
    {
        static Value nul;
        return (type == ARR && i < a.size()) ? a[i] : nul;
    }

    size_t size() const
    {
        return type == ARR ? a.size() : (type == OBJ ? o.size() : 0);
    }

    double Num(double def = 0) const
    {
        return type == NUM ? n : (type == BOOL ? (b ? 1 : 0) : def);
    }

    std::string Str(const std::string& def = "") const
    {
        return type == STR ? s : def;
    }

    bool Bool(bool def = false) const
    {
        return type == BOOL ? b : (type == NUM ? n != 0 : def);
    }
};

class Parser
{
  public:
    explicit Parser(const std::string& text)
        : m_t(text)
    {
    }

    Value Parse()
    {
        Value v = ParseValue();
        Ws();
        if (m_i != m_t.size())
        {
            Fail("trailing characters");
        }
        return v;
    }

  private:
    const std::string& m_t;
    size_t m_i = 0;

    [[noreturn]] void Fail(const std::string& msg)
    {
        throw std::runtime_error("JSON: " + msg + " at offset " + std::to_string(m_i));
    }

    void Ws()
    {
        while (m_i < m_t.size() && (m_t[m_i] == ' ' || m_t[m_i] == '\n' || m_t[m_i] == '\r' ||
                                    m_t[m_i] == '\t'))
        {
            m_i++;
        }
    }

    bool Lit(const char* s)
    {
        size_t n = std::char_traits<char>::length(s);
        if (m_t.compare(m_i, n, s) == 0)
        {
            m_i += n;
            return true;
        }
        return false;
    }

    std::string ParseString()
    {
        if (m_t[m_i] != '"')
        {
            Fail("string expected");
        }
        m_i++;
        std::string out;
        while (m_i < m_t.size() && m_t[m_i] != '"')
        {
            char c = m_t[m_i++];
            if (c == '\\')
            {
                char e = m_t[m_i++];
                switch (e)
                {
                case 'n':
                    out += '\n';
                    break;
                case 't':
                    out += '\t';
                    break;
                case 'r':
                    out += '\r';
                    break;
                case 'b':
                    out += '\b';
                    break;
                case 'f':
                    out += '\f';
                    break;
                case 'u': {
                    unsigned cp = std::stoul(m_t.substr(m_i, 4), nullptr, 16);
                    m_i += 4;
                    if (cp < 0x80)
                    {
                        out += char(cp);
                    }
                    else if (cp < 0x800)
                    {
                        out += char(0xC0 | (cp >> 6));
                        out += char(0x80 | (cp & 0x3F));
                    }
                    else
                    {
                        out += char(0xE0 | (cp >> 12));
                        out += char(0x80 | ((cp >> 6) & 0x3F));
                        out += char(0x80 | (cp & 0x3F));
                    }
                    break;
                }
                default:
                    out += e;
                }
            }
            else
            {
                out += c;
            }
        }
        if (m_i >= m_t.size())
        {
            Fail("unterminated string");
        }
        m_i++;
        return out;
    }

    Value ParseValue()
    {
        Ws();
        if (m_i >= m_t.size())
        {
            Fail("unexpected end");
        }
        Value v;
        char c = m_t[m_i];
        if (c == '{')
        {
            v.type = Value::OBJ;
            m_i++;
            Ws();
            if (m_t[m_i] == '}')
            {
                m_i++;
                return v;
            }
            while (true)
            {
                Ws();
                std::string k = ParseString();
                Ws();
                if (m_t[m_i++] != ':')
                {
                    Fail("':' expected");
                }
                v.o[k] = ParseValue();
                Ws();
                if (m_t[m_i] == ',')
                {
                    m_i++;
                    continue;
                }
                if (m_t[m_i] == '}')
                {
                    m_i++;
                    break;
                }
                Fail("',' or '}' expected");
            }
        }
        else if (c == '[')
        {
            v.type = Value::ARR;
            m_i++;
            Ws();
            if (m_t[m_i] == ']')
            {
                m_i++;
                return v;
            }
            while (true)
            {
                v.a.push_back(ParseValue());
                Ws();
                if (m_t[m_i] == ',')
                {
                    m_i++;
                    continue;
                }
                if (m_t[m_i] == ']')
                {
                    m_i++;
                    break;
                }
                Fail("',' or ']' expected");
            }
        }
        else if (c == '"')
        {
            v.type = Value::STR;
            v.s = ParseString();
        }
        else if (Lit("true"))
        {
            v.type = Value::BOOL;
            v.b = true;
        }
        else if (Lit("false"))
        {
            v.type = Value::BOOL;
        }
        else if (Lit("null"))
        {
            v.type = Value::NUL;
        }
        else
        {
            const char* start = m_t.c_str() + m_i;
            char* end = nullptr;
            v.n = std::strtod(start, &end);
            if (end == start)
            {
                Fail("value expected");
            }
            v.type = Value::NUM;
            m_i += end - start;
        }
        return v;
    }
};

inline Value
ParseFile(const std::string& path)
{
    std::ifstream f(path);
    if (!f)
    {
        throw std::runtime_error("cannot open " + path);
    }
    std::stringstream ss;
    ss << f.rdbuf();
    std::string text = ss.str();
    return Parser(text).Parse();
}

} // namespace mj

#endif
