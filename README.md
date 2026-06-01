# lau-spatial

> QuadTree, GridHash, and SpatialHash — find what's near what, fast. Built for game worlds where millions of entities need to know their neighbors.

## What This Does

Spatial indexing for the **Lau (Layered Agent-UI)** game world. When your game has thousands of entities — players, NPCs, projectiles, particles — checking every pair for proximity is O(n²). Spatial indexing brings it down to O(n log n) or O(n) by dividing space into buckets.

Three data structures, three tradeoffs:
- **QuadTree** — hierarchical, adaptive, great for non-uniform distributions
- **GridHash** — simple uniform grid, fastest for roughly-uniform worlds
- **SpatialHash** — grid + support for entity sizes, best for collision detection

## The Key Idea

In a game world, most entities only interact with nearby entities. A player in the Meadow doesn't need to know about a NPC in the Crystal Caves. Spatial indexing answers "what's within X meters of point Y?" without checking every entity.

This is critical for:
- **Collision detection** — who hit what?
- **Neighbor queries** — which agents can see each other?
- **Vibe propagation** — Lau's emotional AI spreads through proximity
- **Rendering culling** — don't draw what's off-screen

## Install

```bash
cargo add lau-spatial
```

## Quick Start

### QuadTree

```rust
use lau_spatial::{QuadTree, Vec2, BoundingBox};

let mut tree = QuadTree::new(BoundingBox::centered(Vec2::zero(), 100.0));

// Insert entities
tree.insert(Vec2::new(10.0, 20.0), "player")?;
tree.insert(Vec2::new(12.0, 18.0), "npc-1")?;
tree.insert(Vec2::new(50.0, 50.0), "npc-2")?;

// Find everyone within radius 5 of point (11, 19)
let nearby = tree.query_radius(Vec2::new(11.0, 19.0), 5.0);
// Returns: [("player", 10.0, 20.0), ("npc-1", 12.0, 18.0)]
// npc-2 is too far away
```

### GridHash

```rust
use lau_spatial::{GridHash, Vec2};

let mut grid = GridHash::new(10.0); // 10-unit cells

grid.insert(Vec2::new(15.0, 25.0), "entity-a");
grid.insert(Vec2::new(16.0, 24.0), "entity-b");
grid.insert(Vec2::new(95.0, 95.0), "entity-c"); // different cell

// Query a cell
let cell_contents = grid.query_cell(Vec2::new(15.0, 25.0));
// Returns: ["entity-a", "entity-b"]

// Query neighbors (current cell + 8 surrounding)
let neighbors = grid.query_neighbors(Vec2::new(15.0, 25.0));
```

### SpatialHash (supports entity sizes)

```rust
use lau_spatial::{SpatialHash, Vec2};

let mut hash = SpatialHash::new(10.0);

// Entities with bounding radius
hash.insert(Vec2::new(20.0, 20.0), 2.0, "large-entity");  // radius 2
hash.insert(Vec2::new(21.0, 21.0), 0.5, "small-entity");  // radius 0.5

// Large entity spans multiple cells — all are indexed
// Query finds both
let nearby = hash.query_radius(Vec2::new(20.5, 20.5), 3.0);
```

## API Reference

### Vec2

| Method | Description |
|--------|-------------|
| `Vec2::new(x, y)` | Create vector |
| `Vec2::zero()` | Origin |
| `v.length()` | Magnitude |
| `v.normalize()` | Unit vector |
| `v.distance_to(other)` | Euclidean distance |
| `v.dot(other)` | Dot product |

### BoundingBox

| Method | Description |
|--------|-------------|
| `BoundingBox::centered(center, size)` | Square box |
| `BoundingBox::from_corners(min, max)` | Arbitrary rectangle |
| `bb.contains(point)` | Point-in-box test |
| `bb.intersects(other)` | Box-box overlap |
| `bb.center()` / `bb.size()` | Properties |

### QuadTree

| Method | Description |
|--------|-------------|
| `QuadTree::new(bounds)` | Create tree |
| `tree.insert(pos, data)` | Add entity |
| `tree.remove(pos)` | Remove entity |
| `tree.query_radius(center, radius)` | Proximity search |
| `tree.query_box(bounds)` | Rectangle search |
| `tree.len()` | Entity count |
| `tree.clear()` | Remove all |

### GridHash

| Method | Description |
|--------|-------------|
| `GridHash::new(cell_size)` | Create grid |
| `grid.insert(pos, data)` | Add entity to cell |
| `grid.remove(pos)` | Remove |
| `grid.query_cell(pos)` | Same cell |
| `grid.query_neighbors(pos)` | 9-cell neighborhood |
| `grid.query_radius(pos, r)` | Approximate radius search |

### SpatialHash

| Method | Description |
|--------|-------------|
| `SpatialHash::new(cell_size)` | Create hash |
| `hash.insert(pos, radius, data)` | Add sized entity |
| `hash.remove(pos, radius)` | Remove |
| `hash.query_radius(pos, r)` | Find nearby entities |
| `hash.query_potential_collisions()` | All overlapping pairs |

## How It Works

**QuadTree**: Recursively subdivides 2D space into 4 quadrants. Each node holds up to `capacity` entries before splitting. Queries traverse only relevant quadrants. Best for non-uniform distributions (clusters of entities in cities, sparse wilderness).

**GridHash**: Maps continuous coordinates to discrete grid cells via `floor(x/cell_size)`. Each cell is a Vec. Query = hash the cell coordinates. O(1) insertion, O(n/cells) query. Best for uniform distributions.

**SpatialHash**: Like GridHash but entities with radius > cell_size span multiple cells. Insert writes to all overlapped cells. Collision detection = check for entities sharing any cell. Best for variable-sized entities.

All three use `f64` coordinates, `serde` serialization, and zero unsafe code.

## Testing

33 tests covering: insertion/removal, radius queries, box queries, boundary conditions, overlapping entities, large batch performance, serialization, Vec2 arithmetic, BoundingBox operations.

## Part of the Lau Platform

- **lau-git-world** — Git-native game worlds
- **lau-quest** — Quest/mission system
- **lau-biome** — 10 ecological zones
- **lau-spatial** — You are here
- **lau-audio** — Procedural audio
- **lau-scheduler** — Game loop
- **lau-memory-arena** — Entity allocator
- **lau-genealogy** — Lineage tracking
- **lau-recipe** — Crafting recipes

## License

MIT
