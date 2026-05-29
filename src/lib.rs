/// Core arithmetic operations.

/// Adds two numbers.
pub fn add(a: f64, b: f64) -> f64 {
    a + b
}

/// Subtracts b from a.
pub fn subtract(a: f64, b: f64) -> f64 {
    a - b
}

/// Multiplies two numbers.
pub fn multiply(a: f64, b: f64) -> f64 {
    a * b
}

/// Divides a by b.
///
/// # Panics
///
/// Panics if `b` is zero.
pub fn divide(a: f64, b: f64) -> f64 {
    if b == 0.0 {
        panic!("division by zero");
    }
    a / b
}

/// Raises a to the power of b.
pub fn power(a: f64, b: f64) -> f64 {
    a.powf(b)
}
