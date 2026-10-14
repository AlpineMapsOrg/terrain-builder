# Raster store TODO

- Consider extending `sf::validate_index()` beyond rejecting `Inner` after
  additional SF invariants and their required error reporting are defined.
  This is not part of the shared-store refactor.
- Design derived-channel generation (shading, slope, PLaTSA, AO). It is not
  part of the TB builder, which converts one RF into one TB without computing
  channels. A separate tool is expected to produce an RF-like input for the TB
  builder;