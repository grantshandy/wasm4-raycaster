all:
	rustc \
		--target wasm32-unknown-unknown \
		--crate-type cdylib \
		-C panic=abort \
		-C strip=symbols \
		-C codegen-units=1 \
		-C opt-level=z \
		-C lto=true \
		-C link-arg=--import-memory \
		-C link-arg=--initial-memory=65536 \
		-C link-arg=--max-memory=65536 \
		-C link-arg=-zstack-size=14752 \
		-o raycaster.wasm \
		raycaster.rs

	wasm-opt -Oz raycaster.wasm -o raycaster.wasm

	du -b raycaster.wasm

run: all
	wasmstation raycaster.wasm	