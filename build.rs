fn main() {
    embuild::espidf::sys::linkup();
    embuild::build::link_stdcpp();
}
