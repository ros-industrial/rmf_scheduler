#!/bin/bash

(cd script && \
  pybind11-stubgen rmf2_scheduler -o ../
)

