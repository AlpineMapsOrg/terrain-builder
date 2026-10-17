# Raster store design

This directory describes authoritative raster tile storage and the planned
generation of delivery tile pyramids from it. RF import, RF merging and TB
generation are implemented in [tile-foundry](https://github.com/AlpineMapsOrg/tile-foundry);
the tile server remains future work.

## Documents

- [Terminology](terminology.md)
- [Architecture](architecture.md): components, lifecycle, the TB builder and
  the planned tile server
- [Storage format](storage-format.md)
- [TODO](todo.md)

Decisions and their rationale are recorded in the [ADRs](../adr/).
