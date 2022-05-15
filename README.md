# NYCU CA2022 Homework 3

## Result Overview
<img width="935" alt="截圖 2022-05-15 下午2 01 29" src="https://user-images.githubusercontent.com/84212529/168463871-38d00178-4d54-4617-bd30-64a521b4f53d.png">

<img width="934" alt="截圖 2022-05-15 下午2 57 30" src="https://user-images.githubusercontent.com/84212529/168463827-164426da-5ddf-401f-996e-62c0e5e1228c.png">

<img width="935" alt="截圖 2022-05-15 下午2 58 25" src="https://user-images.githubusercontent.com/84212529/168463830-4ffe56cb-3cc4-4fc5-9842-195b364e3b2a.png">

## Dependencies

- [Eigen](https://eigen.tuxfamily.org)
- [glfw](https://github.com/glfw/glfw)
- [glad](https://github.com/Dav1dde/glad)
- [Dear ImGui](https://github.com/ocornut/imgui)


### Dependencies for Windows

Visual Studio

### Dependencies for macOS

Xcode

### Dependencies for Unix-like systems other than macOS with X11

On *Debian* and derivatives like *Ubuntu* and *Linux Mint*

`sudo apt install xorg-dev`

On *Fedora* and derivatives like *Red Hat*

`sudo dnf install libXcursor-devel libXi-devel libXinerama-devel libXrandr-devel`

On *FreeBSD*

`pkg install xorgproto`

## Build instruction

### CMake

Build in release mode
```bash=
cmake -S . -B build -D CMAKE_BUILD_TYPE=Release && cmake --build build --config Release --parallel 8
cd bin
./HW3
```

Build in debug mode
```bash=
cmake -S . -B build -D CMAKE_BUILD_TYPE=Debug
cmake --build build --config Debug --parallel 8
cd bin
./HW3
```

### Visual Studio 2019

- Open `HW3.sln`
- Select config then build (CTRL+SHIFT+B)
- Use F5 to debug or CTRL+F5 to run.
