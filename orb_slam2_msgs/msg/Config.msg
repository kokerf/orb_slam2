# Config for ORB Extractor
# Header (time and frame)
Header header

# Number of features per image
uint16 nFeatures

# Scale factor between levels in the scale pyramid
float32 scaleFactor

# Number of levels in the scale pyramid
uint16 nLevels

# Fast threshold
# Image is divided in a grid. At each cell FAST are extracted imposing a minimum response.
uint16 iniThFAST

uint16 minThFAST

# Succeed flag
bool flag