"""Simple CLI demo for the calculator package."""

from calculator.core import add, subtract, multiply, divide, power


def main() -> None:
    """Run a few arithmetic operations and print results."""
    a, b = 10, 3
    print(f"{a} + {b} = {add(a, b)}")
    print(f"{a} - {b} = {subtract(a, b)}")
    print(f"{a} * {b} = {multiply(a, b)}")
    print(f"{a} / {b} = {divide(a, b):.4f}")
    print(f"{a} ** {b} = {power(a, b)}")


if __name__ == "__main__":
    main()
