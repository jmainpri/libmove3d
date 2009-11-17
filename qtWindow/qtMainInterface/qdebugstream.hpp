#ifndef QDEBUGSTREAM_HPP
#define QDEBUGSTREAM_HPP

#include <iostream>
#include <streambuf>
#include <string>

#include "p3d_sys.h"

class QDebugStream : public std::basic_streambuf<char>
{
public:
    QDebugStream(std::ostream &stream, QTextEdit* text_edit);
    ~QDebugStream();

protected:
    virtual char overflow(char v);
    virtual std::streamsize xsputn(const char *p, std::streamsize n);

private:
    std::ostream &m_stream;
    std::streambuf *m_old_buf;
    std::string m_string;
    QTextEdit* log_window;
};


#endif // QDEBUGSTREAM_H
