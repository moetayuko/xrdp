//
// Created by cheesekun on 2/28/22.
//

#ifndef ULALACA_SYSTEMCALLEXCEPTION_HPP
#define ULALACA_SYSTEMCALLEXCEPTION_HPP

#include <string>
#include <exception>


class SystemCallException: public std::exception {
public:
    explicit SystemCallException(
            int _errno,
            std::string funcName
    );
    ~SystemCallException() override = default;

    int getErrno() const;
    const std::string &getMessage() const;

    const char* what() const noexcept override;

private:
    int _errno;
    std::string _message;

    std::string _funcName;

    std::string _what;
};


#endif //ULALACA_SYSTEMCALLEXCEPTION_HPP
