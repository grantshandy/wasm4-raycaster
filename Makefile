all:
	cargo build --release

	wasm-opt -Oz target/wasm32-unknown-unknown/release/raycaster.wasm \
		-o target/wasm32-unknown-unknown/release/raycaster.wasm

size: all
	du -bh target/wasm32-unknown-unknown/release/raycaster.wasm

run: all
	w4 run-native target/wasm32-unknown-unknown/release/raycaster.wasm