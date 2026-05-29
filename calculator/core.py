"""Core arithmetic operations with type hints and docstrings."""

from typing import Union

Number = Union[int, float]


def add(a: Number, b: Number) -> Number:
    """Return the sum of a and b."""
    return a + b


def subtract(a: Number, b: Number) -> Number:
    """Return the difference a - b."""
    return a - b


def multiply(a: Number, b: Number) -> Number:
    """Return the product of a and b."""
    return a * b


def divide(a: Number, b: Number) -> float:
    """Return a divided by b.

    Raises:
        ZeroDivisionError: If b is zero.
    """
    if b == 0:
        raise ZeroDivisionError("division by zero")
    return a / b


def power(a: Number, b: Number) -> Number:
    """Return a raised to the power of b."""
    return a ** b
