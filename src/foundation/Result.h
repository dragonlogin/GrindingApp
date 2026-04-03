#pragma once

#include <stdexcept>
#include <utility>

#include "Error.h"

namespace foundation {

template <typename T>
class [[nodiscard]] Result {
public:
    static Result Ok(T value)
    {
        return Result(std::move(value));
    }

    static Result Fail(Error error)
    {
        return Result(std::move(error));
    }

    bool ok() const { return ok_; }
    explicit operator bool() const { return ok_; }

    const T& value() const
    {
        if (!ok_) {
            throw std::logic_error("Result does not contain a value");
        }
        return value_;
    }

    T& value()
    {
        if (!ok_) {
            throw std::logic_error("Result does not contain a value");
        }
        return value_;
    }

    const Error& error() const
    {
        if (ok_) {
            throw std::logic_error("Result does not contain an error");
        }
        return error_;
    }

private:
    explicit Result(T value)
        : ok_(true), value_(std::move(value))
    {
    }

    explicit Result(Error error)
        : ok_(false), error_(std::move(error))
    {
    }

private:
    bool ok_ = false;
    T value_{};
    Error error_{};
};

template <>
class Result<void> {
public:
    static Result Ok()
    {
        return Result(true, {});
    }

    static Result Fail(Error error)
    {
        return Result(false, std::move(error));
    }

    bool ok() const { return ok_; }
    explicit operator bool() const { return ok_; }

    const Error& error() const
    {
        if (ok_) {
            throw std::logic_error("Result does not contain an error");
        }
        return error_;
    }

private:
    Result(bool ok, Error error)
        : ok_(ok), error_(std::move(error))
    {
    }

private:
    bool ok_ = false;
    Error error_{};
};

} // namespace foundation
