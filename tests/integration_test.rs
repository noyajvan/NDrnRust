/// Integration tests for the calculator library.

use calculator;

#[test]
fn test_add_positive() {
    assert_eq!(calculator::add(2.0, 3.0), 5.0);
}

#[test]
fn test_add_negative() {
    assert_eq!(calculator::add(-1.0, -1.0), -2.0);
}

#[test]
fn test_add_float() {
    assert_eq!(calculator::add(1.5, 2.5), 4.0);
}

#[test]
fn test_subtract_positive() {
    assert_eq!(calculator::subtract(5.0, 3.0), 2.0);
}

#[test]
fn test_subtract_negative_result() {
    assert_eq!(calculator::subtract(3.0, 5.0), -2.0);
}

#[test]
fn test_subtract_float() {
    assert!((calculator::subtract(5.5, 2.2) - 3.3).abs() < 1e-10);
}

#[test]
fn test_multiply_positive() {
    assert_eq!(calculator::multiply(4.0, 5.0), 20.0);
}

#[test]
fn test_multiply_zero() {
    assert_eq!(calculator::multiply(0.0, 100.0), 0.0);
}

#[test]
fn test_multiply_negative() {
    assert_eq!(calculator::multiply(-3.0, 4.0), -12.0);
}

#[test]
fn test_divide_positive() {
    assert_eq!(calculator::divide(10.0, 2.0), 5.0);
}

#[test]
fn test_divide_float_result() {
    let result = calculator::divide(7.0, 3.0);
    let expected = 7.0 / 3.0;
    assert!((result - expected).abs() < 1e-10);
}

#[test]
#[should_panic(expected = "division by zero")]
fn test_divide_by_zero() {
    calculator::divide(1.0, 0.0);
}

#[test]
fn test_power_positive() {
    assert_eq!(calculator::power(2.0, 3.0), 8.0);
}

#[test]
fn test_power_zero_exponent() {
    assert_eq!(calculator::power(5.0, 0.0), 1.0);
}

#[test]
fn test_power_negative_exponent() {
    assert!((calculator::power(2.0, -1.0) - 0.5).abs() < 1e-10);
}

#[test]
fn test_power_float_exponent() {
    assert!((calculator::power(4.0, 0.5) - 2.0).abs() < 1e-10);
}
