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

run: all
	du -sh ./raycaster.wasm
	w4 run-native ./raycaster.wasm	