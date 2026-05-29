"""Tests for calculator.core."""

import pytest
from calculator.core import add, subtract, multiply, divide


class TestAdd:
    def test_positive(self) -> None:
        assert add(2, 3) == 5

    def test_negative(self) -> None:
        assert add(-1, -1) == -2

    def test_float(self) -> None:
        assert add(1.5, 2.5) == 4.0


class TestSubtract:
    def test_positive(self) -> None:
        assert subtract(5, 3) == 2

    def test_negative_result(self) -> None:
        assert subtract(3, 5) == -2

    def test_float(self) -> None:
        assert subtract(5.5, 2.2) == pytest.approx(3.3)


class TestMultiply:
    def test_positive(self) -> None:
        assert multiply(4, 5) == 20

    def test_zero(self) -> None:
        assert multiply(0, 100) == 0

    def test_negative(self) -> None:
        assert multiply(-3, 4) == -12


class TestDivide:
    def test_positive(self) -> None:
        assert divide(10, 2) == 5.0

    def test_float_result(self) -> None:
        assert divide(7, 3) == pytest.approx(2.3333333333333335)

    def test_division_by_zero(self) -> None:
        with pytest.raises(ZeroDivisionError):
            divide(1, 0)
