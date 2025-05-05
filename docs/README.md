# Documentation

### Sphinx Documentation

Install dependencies.

```bash
sudo apt install python3-pip python3-dev
pip3 install --user --upgrade setuptools
pip3 install --user --upgrade sphinx sphinx-rtd-theme myst_parser recommonmark sphinxcontrib-jquery
```

Generate the documentation.

```bash
cd $COLCON_WS/src/rmf_scheduler/
make -C docs/sphinx html
```

Open the documentation with your favourite web browser.

```bash
firefox docs/sphinx/build/html/index.html
```

### API Documentation

Install Doxygen.

```bash
sudo apt install doxygen graphviz
```

Generate API documentation.

```bash
cd $COLCON_WS/src/rmf_scheduler/
mkdir -p docs/doxygen/build
cd .doxygen
doxygen
```

Open the documentation with your favourite web browser.

```
cd ..
firefox docs/doxygen/build/html/index.html
```
