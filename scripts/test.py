import pandas as pd
from pathlib import Path

print(__file__)
HDF_PATH = Path(__file__).parent / ".." / "hdf5_data" / "low_dim_v141.hdf5"
print(HDF_PATH)
f = pd.read_hdf(HDF_PATH)
print(f)