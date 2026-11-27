# Widening the BVH

## Goal

Rework `src/bvh_builder_old/widen.h` into `src/bvh_builder`. It collapses the binary tree the builders produce into one with up to `arity` children per node, by the Ylitie et al. (2017) dynamic program.

The old file cannot be ported directly. It assumes the whole tree is resident and resolves references by random access, and it holds a DP table per node for the entire tree at once.

## What widen may assume

Nothing about how the input is batched. Not that children precede parents, not that a batch holds a subtree, and nothing PLOC-specific such as round structure or level-uniform batches.

The one thing it does rely on is that a node's header equals `bvh::merge_headers` over its children's headers. That is how `frontier::merge_entry` builds every inner header, and the fold is associative and commutative in all four fields, so it also holds for any regrouping of the same leaves.

## Design

Two traversals of the tree, with a sidecar stream between them. No mailbox, no cost sidecar, no random access on the write side.

1. **Pass 1** walks the input tree in post-order, following child references. At each node it evaluates, for every slot count the node might be offered, what its subtree costs and what it would do. A node's cost row is consumed by its parent the moment the child returns, so cost rows are stack state and never stored. The pass appends one record per node to the sidecar, in post-order.
2. **Pass 2** reads that sidecar backwards. The reverse of a post-order stream visits a node before its subtree, which is the order the emit descends in, so the cursor moves monotonically and never seeks. It carries the offered slot count on the stack, recurses, and emits one wide node per kept node once its wide children exist.

Pass 2 never touches the input tree. That is the point of the sidecar carrying everything: the only unpredictable cost in the design, the miss rate of a descent against disk-backed batches, is confined to pass 1.

### What the sidecar carries

Per node, in post-order:

- For an inner node, its split row: `Split { left_slots, kept }` per slot count, ragged.
- For a leaf, its `dag::Id` and its `NodeHeader`.

No child references, because the descent order is the stream order. No subtree sizes, because nothing seeks. No inner headers, because pass 2 merges its wide children's headers back into one and gets the original bit-for-bit.

### Why the rows are ragged

A subtree with `m` leaves costs nothing once it is offered `m` slots or more, because every leaf takes a slot of its own and no node needs keeping. So a node's row is `min(arity, m - 1)` long, and a read past the end is zero.

Half the inner nodes sit directly above two leaves and have a one-entry row. That keeps the sidecar at roughly `2N` bytes of split rows for `N` leaves, **independent of arity**. Fixed-width rows would be `2 * arity * N / 2`.

Row lengths are one `uint8_t` per node, hence `arity <= 255`.

### The batch cache

Pass 1's descent follows references into arbitrary batches, so it needs a resident set of `bvh::Batch` keyed by batch index, bounded by `max_resident_bytes` and evicting least recently used. Read-through only; pass 1 writes nothing but the sidecar stream.

Its hit rate is the one number the design cannot predict. It depends on how well the builder's layout matches a depth-first walk, which is near-perfect for LBVH post-order and unknown for PLOC. Small batches are what make it affordable: halving the batch size halves the cost of a miss and doubles how many batches stay resident, so the two effects compound.

## Steps

Each lands on its own.

1. **Lower `DEFAULT_MAX_BATCH_BYTES`.** 100 MB is a very large unit for a streaming pipeline. Around 1 MB keeps the sequential passes saturating throughput while making a cache miss cheap. Worth doing on its own merits, and it is what makes step 3 viable.

2. **Delete the mailbox architecture.** `widen/Mailbox.h`, `widen/assign_wide_parents.h`, `widen/write_wide_nodes.h`, `CostBatch` and `CostStorage` from `tabulate_splits.h`, and the untracked `src/bvh_builder/BatchWriter.h` left over from the rejected `ItemBatch` writer. `unittests/bvh_builder/mailbox.cpp` goes with them, which needs a line removed from `unittests/CMakeLists.txt` -- ask first.

3. **The traversal helper.** A cache over `bvh::Storage` resolving a `bvh::Reference` to a `const bvh::Node &`, plus the post-order descent driving it. Tested on a tree spanning several batches with a budget that forces eviction.

4. **Pass 1.** The Ylitie dynamic program at a node, over its children's returned rows, appending split rows to the sidecar. The four-leaf arity-4 tree worked through by hand is the case that decides whether the design holds:

   ```
           R
         /   \
        A     B
       / \   / \
      0   1 2   3
   ```

   R keeps its slot, A and B are dropped, and all four leaves become R's children.

5. **Reverse iteration on `RowBuffer`.** Pass 2 reads rows back to front. `_lengths` is already a separate vector, so this is a cursor walking it backwards from the end of `_rows`.

6. **Pass 2.** The backward descent, emitting through `bvh::BatchWriter` and merging headers on the way up. Returns the new root.

7. **`--arity` in `cli.h`, and the call in `main.cpp`.**

8. **A `docs/` writeup.** The streaming reformulation of the dynamic program is the part worth recording conceptually.

## Vocabulary

Settled after a long argument, worth keeping.

- **slots**: how many of a parent's child slots a subtree may occupy. Not "budget".
- Pass names say what comes out, not how it works. `tabulate_splits` does not choose anything, it builds the table pass 2 chooses from.
- `Split { left_slots, kept }`, one per node per slot count.

## Done

- `build/merge_cost.h` gained `compute_node_cost` for a 4-D box and for a `NodeHeader`; `compute_merge_cost` now goes through it.
- `storage/` holds `BaseStorage.h`, `BatchStorage.h`, `batch_limits.h` and `BaseBatchWriter.h`.
- `BaseBatchWriter` owns the pending batch, the write-on-destruction guard, `flush`, `flush_if_full` and the size threshold. `bvh::BatchWriter` and `frontier::BatchWriter` are `final` derivations supplying only how their batch is born and how big it is.
- `widen/RowBuffer.h` implemented, with tests in `unittests/bvh_builder/row_buffer.cpp`.

## Process

API first, then tests, then bodies. One test file per header, named after it, tagged `[widen]`.