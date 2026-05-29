/// Simple CLI demo for the calculator library.

mod lib;

fn main() {
    let a = 10.0;
    let b = 3.0;

    println!("{} + {} = {}", a, b, lib::add(a, b));
    println!("{} - {} = {}", a, b, lib::subtract(a, b));
    println!("{} * {} = {}", a, b, lib::multiply(a, b));
    println!("{} / {} = {:.4}", a, b, lib::divide(a, b));
    println!("{} ** {} = {}", a, b, lib::power(a, b));
}
