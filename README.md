**WIP**

## Build

### Normal

```pwsh
./wamrc.exe --target=x86_64 --cpu=x86-64-v3 --opt-level=3 --size-level=0 --enable-simd --format=aot -o x86_64\HavokPhysics.aot HavokPhysics.wasm
```

### PGO

Run `apply-pgo` action to download artifact, then:

```pwsh
./wamrc.exe --target=x86_64 --cpu=x86-64-v3 --opt-level=3 --size-level=1 --enable-simd --use-prof-file=HavokPhysics.profdata --format=aot -o x86_64\HavokPhysicsPGO.aot HavokPhysics.wasm
```