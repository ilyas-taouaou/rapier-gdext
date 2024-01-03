use godot::init::{gdextension, ExtensionLibrary};

mod backend;
mod frontend;
mod middleware;

struct MyExtension;

#[gdextension]
unsafe impl ExtensionLibrary for MyExtension {}
