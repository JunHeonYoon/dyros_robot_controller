# Build and Run

## Build the Library

```bash
cd dyros_robot_controller
mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX:PATH=<custom-folder> ../
make
make install
```

Add to `~/.bashrc`:

```bash
export dyros_robot_controller_DIR="<custom-folder>"
export LD_LIBRARY_PATH="$dyros_robot_controller_DIR/lib:$LD_LIBRARY_PATH"
```

## Build C++ Examples

```bash
cd dyros_robot_controller/examples/C++
mkdir -p build
cd build
cmake ..
cmake --build . -j
```

## Run C++ Examples

```bash
cd dyros_robot_controller/examples/C++/build
./dyros_robot_controller_example fr3
./dyros_robot_controller_example xls
./dyros_robot_controller_example fr3_xls
```

## Run Python Examples

```bash
cd dyros_robot_controller/examples/python
python3 dyros_robot_controller_example.py --robot_name fr3
python3 dyros_robot_controller_example.py --robot_name xls
python3 dyros_robot_controller_example.py --robot_name fr3_xls
```

## Build the Documentation Site

```bash
cd dyros_robot_controller
python3 -m pip install -r docs/requirements.txt
mkdocs build --strict
```

## Serve the Documentation Locally

```bash
mkdocs serve
```

Open `http://127.0.0.1:8000/dyros_robot_controller/` when `site_url` is enabled, or use the URL printed by MkDocs.
