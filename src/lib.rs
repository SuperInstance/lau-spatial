//! # lau-spatial
//!
//! Spatial indexing library for game worlds — efficient neighbor queries for
//! vibe propagation, collision, and agent sensing.
//!
//! Provides a [`QuadTree`] for hierarchical spatial indexing, a [`GridHash`]
//! for simple uniform grid hashing, and a [`SpatialHash`] that combines grid
//! hashing with support for entry sizes.

use serde::{Deserialize, Serialize};
use std::collections::HashMap;

// ─── Vec2 ────────────────────────────────────────────────────────────────────

/// 2D vector with `f64` components.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Vec2 {
    pub x: f64,
    pub y: f64,
}

impl Vec2 {
    pub const fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }

    pub const fn zero() -> Self {
        Self { x: 0.0, y: 0.0 }
    }

    pub fn dot(&self, other: &Self) -> f64 {
        self.x * other.x + self.y * other.y
    }

    pub fn length(&self) -> f64 {
        self.dot(self).sqrt()
    }

    pub fn normalize(&self) -> Self {
        let len = self.length();
        if len == 0.0 {
            Self::zero()
        } else {
            *self / len
        }
    }

    pub fn distance(&self, other: &Self) -> f64 {
        (*self - *other).length()
    }

    pub fn lerp(&self, other: &Self, t: f64) -> Self {
        *self + (*other - *self) * t
    }

    pub fn angle(&self) -> f64 {
        self.y.atan2(self.x)
    }

    pub fn rotate(&self, angle: f64) -> Self {
        let cos = angle.cos();
        let sin = angle.sin();
        Self {
            x: self.x * cos - self.y * sin,
            y: self.x * sin + self.y * cos,
        }
    }
}

impl std::ops::Add for Vec2 {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Self { x: self.x + rhs.x, y: self.y + rhs.y }
    }
}

impl std::ops::Sub for Vec2 {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        Self { x: self.x - rhs.x, y: self.y - rhs.y }
    }
}

impl std::ops::Mul<f64> for Vec2 {
    type Output = Self;
    fn mul(self, rhs: f64) -> Self {
        Self { x: self.x * rhs, y: self.y * rhs }
    }
}

impl std::ops::Div<f64> for Vec2 {
    type Output = Self;
    fn div(self, rhs: f64) -> Self {
        Self { x: self.x / rhs, y: self.y / rhs }
    }
}

impl std::ops::Neg for Vec2 {
    type Output = Self;
    fn neg(self) -> Self {
        Self { x: -self.x, y: -self.y }
    }
}

// ─── AABB ────────────────────────────────────────────────────────────────────

/// Axis-aligned bounding box.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct AABB {
    pub min: Vec2,
    pub max: Vec2,
}

impl AABB {
    pub fn new(min: Vec2, max: Vec2) -> Self {
        Self { min, max }
    }

    pub fn from_center(center: Vec2, half_size: Vec2) -> Self {
        Self {
            min: center - half_size,
            max: center + half_size,
        }
    }

    pub fn contains(&self, point: Vec2) -> bool {
        point.x >= self.min.x
            && point.x <= self.max.x
            && point.y >= self.min.y
            && point.y <= self.max.y
    }

    pub fn intersects(&self, other: &Self) -> bool {
        self.min.x <= other.max.x
            && self.max.x >= other.min.x
            && self.min.y <= other.max.y
            && self.max.y >= other.min.y
    }

    pub fn center(&self) -> Vec2 {
        (self.min + self.max) * 0.5
    }

    pub fn area(&self) -> f64 {
        let d = self.max - self.min;
        d.x * d.y
    }

    pub fn merge(&self, other: &Self) -> Self {
        Self {
            min: Vec2::new(self.min.x.min(other.min.x), self.min.y.min(other.min.y)),
            max: Vec2::new(self.max.x.max(other.max.x), self.max.y.max(other.max.y)),
        }
    }

    pub fn expand(&self, point: Vec2) -> Self {
        Self {
            min: Vec2::new(self.min.x.min(point.x), self.min.y.min(point.y)),
            max: Vec2::new(self.max.x.max(point.x), self.max.y.max(point.y)),
        }
    }
}

// ─── SpatialEntry ────────────────────────────────────────────────────────────

/// An entry stored in a spatial index.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SpatialEntry {
    pub id: u64,
    pub pos: Vec2,
    pub bounds: AABB,
    pub data: String,
}

// ─── QuadTree ────────────────────────────────────────────────────────────────

/// Hierarchical quad-tree spatial index.
pub struct QuadTree {
    bounds: AABB,
    capacity: usize,
    entries: Vec<SpatialEntry>,
    children: Option<Box<[QuadTree; 4]>>,
    len: usize,
}

impl QuadTree {
    /// Create a new quad-tree covering `bounds` with leaf capacity `capacity`.
    pub fn new(bounds: AABB, capacity: usize) -> Self {
        assert!(capacity > 0, "capacity must be > 0");
        Self {
            bounds,
            capacity,
            entries: Vec::with_capacity(capacity),
            children: None,
            len: 0,
        }
    }

    /// Insert an entry. Returns `false` if the entry is outside bounds.
    pub fn insert(&mut self, entry: SpatialEntry) -> bool {
        if !self.bounds.contains(entry.pos) {
            return false;
        }

        self.len += 1;

        if self.children.is_none() && self.entries.len() < self.capacity {
            self.entries.push(entry);
            return true;
        }

        if self.children.is_none() {
            self.subdivide();
        }

        // Try to insert into the appropriate child.
        if let Some(children) = &mut self.children {
            for child in children.iter_mut() {
                if child.insert(entry.clone()) {
                    return true;
                }
            }
        }

        // Fallback: keep in this node if it doesn't fit neatly into a child.
        self.entries.push(entry);
        true
    }

    fn subdivide(&mut self) {
        let half = (self.bounds.max - self.bounds.min) * 0.5;
        let min = self.bounds.min;

        let bl = min;
        let br = Vec2::new(min.x + half.x, min.y);
        let tl = Vec2::new(min.x, min.y + half.y);
        let tr = min + half;

        let sw_bounds = AABB::new(bl, bl + half);
        let se_bounds = AABB::new(br, br + half);
        let nw_bounds = AABB::new(tl, tl + half);
        let ne_bounds = AABB::new(tr, tr + half);

        let cap = self.capacity;

        self.children = Some(Box::new([
            QuadTree::new(nw_bounds, cap),
            QuadTree::new(ne_bounds, cap),
            QuadTree::new(sw_bounds, cap),
            QuadTree::new(se_bounds, cap),
        ]));

        // Re-distribute existing entries.
        let old = std::mem::take(&mut self.entries);
        for e in old {
            let mut placed = false;
            if let Some(children) = &mut self.children {
                for child in children.iter_mut() {
                    if child.insert(e.clone()) {
                        placed = true;
                        break;
                    }
                }
            }
            if !placed {
                self.entries.push(e);
            }
        }
    }

    /// Query entries at an exact point.
    pub fn query_point(&self, point: Vec2) -> Vec<&SpatialEntry> {
        if !self.bounds.contains(point) {
            return Vec::new();
        }
        let mut result = Vec::new();
        self.query_point_inner(point, &mut result);
        result
    }

    fn query_point_inner<'a>(&'a self, point: Vec2, result: &mut Vec<&'a SpatialEntry>) {
        for e in &self.entries {
            if e.pos == point || e.bounds.contains(point) {
                result.push(e);
            }
        }
        if let Some(children) = &self.children {
            for child in children.iter() {
                if child.bounds.contains(point) {
                    child.query_point_inner(point, result);
                }
            }
        }
    }

    /// Query all entries whose bounds intersect the given AABB.
    pub fn query_range(&self, bounds: AABB) -> Vec<&SpatialEntry> {
        if !self.bounds.intersects(&bounds) {
            return Vec::new();
        }
        let mut result = Vec::new();
        self.query_range_inner(&bounds, &mut result);
        result
    }

    fn query_range_inner<'a>(&'a self, bounds: &AABB, result: &mut Vec<&'a SpatialEntry>) {
        for e in &self.entries {
            if bounds.intersects(&e.bounds) || bounds.contains(e.pos) {
                result.push(e);
            }
        }
        if let Some(children) = &self.children {
            for child in children.iter() {
                if child.bounds.intersects(bounds) {
                    child.query_range_inner(bounds, result);
                }
            }
        }
    }

    /// Query all entries within `radius` of `center`.
    pub fn query_radius(&self, center: Vec2, radius: f64) -> Vec<&SpatialEntry> {
        let half = Vec2::new(radius, radius);
        let search_bounds = AABB::from_center(center, half);
        let candidates = self.query_range(search_bounds);
        candidates
            .into_iter()
            .filter(|e| center.distance(&e.pos) <= radius)
            .collect()
    }

    /// Find the `n` nearest entries to `point`, returned with their distances (sorted nearest first).
    pub fn nearest(&self, point: Vec2, n: usize) -> Vec<(&SpatialEntry, f64)> {
        let mut candidates: Vec<(&SpatialEntry, f64)> = Vec::new();
        self.collect_all(point, &mut candidates);
        candidates.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));
        candidates.truncate(n);
        candidates
    }

    fn collect_all<'a>(&'a self, point: Vec2, out: &mut Vec<(&'a SpatialEntry, f64)>) {
        for e in &self.entries {
            let d = point.distance(&e.pos);
            out.push((e, d));
        }
        if let Some(children) = &self.children {
            for child in children.iter() {
                child.collect_all(point, out);
            }
        }
    }

    /// Total number of entries in the tree.
    pub fn len(&self) -> usize {
        self.len
    }

    /// Whether the tree is empty.
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Remove all entries.
    pub fn clear(&mut self) {
        self.entries.clear();
        self.children = None;
        self.len = 0;
    }
}

// ─── GridHash ────────────────────────────────────────────────────────────────

/// Simple uniform grid spatial hash.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GridHash {
    pub cell_size: f64,
    pub cells: HashMap<(i32, i32), Vec<SpatialEntry>>,
}

impl GridHash {
    pub fn new(cell_size: f64) -> Self {
        assert!(cell_size > 0.0, "cell_size must be > 0");
        Self {
            cell_size,
            cells: HashMap::new(),
        }
    }

    /// Compute the cell key for a position.
    pub fn cell_of(&self, pos: Vec2) -> (i32, i32) {
        (
            (pos.x / self.cell_size).floor() as i32,
            (pos.y / self.cell_size).floor() as i32,
        )
    }

    /// Insert an entry into the grid cell corresponding to its position.
    pub fn insert(&mut self, entry: SpatialEntry) {
        let cell = self.cell_of(entry.pos);
        self.cells.entry(cell).or_default().push(entry);
    }

    /// Get all entries in a specific cell.
    pub fn query_cell(&self, cell: (i32, i32)) -> Vec<&SpatialEntry> {
        self.cells
            .get(&cell)
            .map(|v| v.iter().collect())
            .unwrap_or_default()
    }

    /// Query all entries within `radius` of `center`.
    pub fn query_radius(&self, center: Vec2, radius: f64) -> Vec<&SpatialEntry> {
        let min_cell = self.cell_of(Vec2::new(center.x - radius, center.y - radius));
        let max_cell = self.cell_of(Vec2::new(center.x + radius, center.y + radius));

        let mut result = Vec::new();
        for cx in min_cell.0..=max_cell.0 {
            for cy in min_cell.1..=max_cell.1 {
                if let Some(entries) = self.cells.get(&(cx, cy)) {
                    for e in entries {
                        if center.distance(&e.pos) <= radius {
                            result.push(e);
                        }
                    }
                }
            }
        }
        result
    }

    /// Total number of entries across all cells.
    pub fn len(&self) -> usize {
        self.cells.values().map(|v| v.len()).sum()
    }

    /// Whether the grid is empty.
    pub fn is_empty(&self) -> bool {
        self.cells.values().all(|v| v.is_empty())
    }
}

// ─── SpatialHash ─────────────────────────────────────────────────────────────

/// Grid-based spatial hash with support for entry sizes (inserts into all cells
/// covered by the entry's AABB).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SpatialHash {
    pub cell_size: f64,
    pub cells: HashMap<(i32, i32), Vec<SpatialEntry>>,
}

impl SpatialHash {
    pub fn new(cell_size: f64) -> Self {
        assert!(cell_size > 0.0, "cell_size must be > 0");
        Self {
            cell_size,
            cells: HashMap::new(),
        }
    }

    fn cell_of(&self, pos: Vec2) -> (i32, i32) {
        (
            (pos.x / self.cell_size).floor() as i32,
            (pos.y / self.cell_size).floor() as i32,
        )
    }

    /// Insert an entry into all cells that its bounds overlap.
    pub fn insert(&mut self, entry: SpatialEntry) {
        let min_cell = self.cell_of(entry.bounds.min);
        let max_cell = self.cell_of(entry.bounds.max);

        for cx in min_cell.0..=max_cell.0 {
            for cy in min_cell.1..=max_cell.1 {
                self.cells
                    .entry((cx, cy))
                    .or_default()
                    .push(entry.clone());
            }
        }
    }

    /// Query all entries in the cell containing `point`.
    pub fn query_point(&self, point: Vec2) -> Vec<&SpatialEntry> {
        let cell = self.cell_of(point);
        self.query_cell(cell)
    }

    /// Get all entries in a specific cell.
    pub fn query_cell(&self, cell: (i32, i32)) -> Vec<&SpatialEntry> {
        self.cells
            .get(&cell)
            .map(|v| v.iter().collect())
            .unwrap_or_default()
    }

    /// Query all entries whose bounds intersect the given AABB.
    pub fn query_range(&self, bounds: AABB) -> Vec<&SpatialEntry> {
        let min_cell = self.cell_of(bounds.min);
        let max_cell = self.cell_of(bounds.max);

        let mut seen = std::collections::HashSet::new();
        let mut result = Vec::new();

        for cx in min_cell.0..=max_cell.0 {
            for cy in min_cell.1..=max_cell.1 {
                if let Some(entries) = self.cells.get(&(cx, cy)) {
                    for e in entries {
                        if seen.insert(e.id) && bounds.intersects(&e.bounds) {
                            result.push(e);
                        }
                    }
                }
            }
        }
        result
    }

    /// Query all entries within `radius` of `center`.
    pub fn query_radius(&self, center: Vec2, radius: f64) -> Vec<&SpatialEntry> {
        let half = Vec2::new(radius, radius);
        let search = AABB::from_center(center, half);
        let candidates = self.query_range(search);
        candidates
            .into_iter()
            .filter(|e| center.distance(&e.pos) <= radius)
            .collect()
    }

    /// Total unique entries (approximated by counting; dedup by id).
    pub fn len(&self) -> usize {
        let mut ids = std::collections::HashSet::new();
        for v in self.cells.values() {
            for e in v {
                ids.insert(e.id);
            }
        }
        ids.len()
    }

    /// Whether the spatial hash is empty.
    pub fn is_empty(&self) -> bool {
        self.cells.values().all(|v| v.is_empty())
    }
}

// ─── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn make_entry(id: u64, x: f64, y: f64) -> SpatialEntry {
        let pos = Vec2::new(x, y);
        let half = Vec2::new(0.5, 0.5);
        SpatialEntry {
            id,
            pos,
            bounds: AABB::from_center(pos, half),
            data: format!("entry-{id}"),
        }
    }

    fn make_entry_with_size(id: u64, x: f64, y: f64, half: f64) -> SpatialEntry {
        let pos = Vec2::new(x, y);
        let hs = Vec2::new(half, half);
        SpatialEntry {
            id,
            pos,
            bounds: AABB::from_center(pos, hs),
            data: format!("entry-{id}"),
        }
    }

    // ── Vec2 tests ──

    #[test]
    fn vec2_new_and_zero() {
        let v = Vec2::new(3.0, 4.0);
        assert_eq!(v.x, 3.0);
        assert_eq!(v.y, 4.0);
        let z = Vec2::zero();
        assert_eq!(z.x, 0.0);
        assert_eq!(z.y, 0.0);
    }

    #[test]
    fn vec2_dot_and_length() {
        let v = Vec2::new(3.0, 4.0);
        assert_eq!(v.dot(&v), 25.0);
        assert!((v.length() - 5.0).abs() < 1e-10);
    }

    #[test]
    fn vec2_normalize() {
        let v = Vec2::new(3.0, 4.0).normalize();
        assert!((v.length() - 1.0).abs() < 1e-10);
        let z = Vec2::zero().normalize();
        assert_eq!(z, Vec2::zero());
    }

    #[test]
    fn vec2_distance() {
        let a = Vec2::new(0.0, 0.0);
        let b = Vec2::new(3.0, 4.0);
        assert!((a.distance(&b) - 5.0).abs() < 1e-10);
    }

    #[test]
    fn vec2_lerp() {
        let a = Vec2::new(0.0, 0.0);
        let b = Vec2::new(10.0, 20.0);
        let mid = a.lerp(&b, 0.5);
        assert!((mid.x - 5.0).abs() < 1e-10);
        assert!((mid.y - 10.0).abs() < 1e-10);
    }

    #[test]
    fn vec2_angle_and_rotate() {
        let v = Vec2::new(1.0, 0.0);
        let rotated = v.rotate(std::f64::consts::FRAC_PI_2);
        assert!((rotated.x).abs() < 1e-10);
        assert!((rotated.y - 1.0).abs() < 1e-10);

        let angle = Vec2::new(1.0, 0.0).angle();
        assert!(angle.abs() < 1e-10);
    }

    #[test]
    fn vec2_operators() {
        let a = Vec2::new(1.0, 2.0);
        let b = Vec2::new(3.0, 4.0);
        assert_eq!(a + b, Vec2::new(4.0, 6.0));
        assert_eq!(b - a, Vec2::new(2.0, 2.0));
        assert_eq!(a * 3.0, Vec2::new(3.0, 6.0));
    }

    // ── AABB tests ──

    #[test]
    fn aabb_contains() {
        let aabb = AABB::new(Vec2::new(0.0, 0.0), Vec2::new(10.0, 10.0));
        assert!(aabb.contains(Vec2::new(5.0, 5.0)));
        assert!(aabb.contains(Vec2::new(0.0, 0.0)));
        assert!(aabb.contains(Vec2::new(10.0, 10.0)));
        assert!(!aabb.contains(Vec2::new(11.0, 5.0)));
    }

    #[test]
    fn aabb_intersects() {
        let a = AABB::new(Vec2::new(0.0, 0.0), Vec2::new(10.0, 10.0));
        let b = AABB::new(Vec2::new(5.0, 5.0), Vec2::new(15.0, 15.0));
        let c = AABB::new(Vec2::new(11.0, 11.0), Vec2::new(20.0, 20.0));
        assert!(a.intersects(&b));
        assert!(!a.intersects(&c));
    }

    #[test]
    fn aabb_center_area() {
        let aabb = AABB::new(Vec2::new(0.0, 0.0), Vec2::new(10.0, 5.0));
        assert_eq!(aabb.center(), Vec2::new(5.0, 2.5));
        assert!((aabb.area() - 50.0).abs() < 1e-10);
    }

    #[test]
    fn aabb_merge_expand() {
        let a = AABB::new(Vec2::new(0.0, 0.0), Vec2::new(5.0, 5.0));
        let b = AABB::new(Vec2::new(3.0, 3.0), Vec2::new(10.0, 8.0));
        let merged = a.merge(&b);
        assert_eq!(merged.min, Vec2::new(0.0, 0.0));
        assert_eq!(merged.max, Vec2::new(10.0, 8.0));

        let expanded = a.expand(Vec2::new(12.0, -1.0));
        assert_eq!(expanded.min, Vec2::new(0.0, -1.0));
        assert_eq!(expanded.max, Vec2::new(12.0, 5.0));
    }

    #[test]
    fn aabb_from_center() {
        let aabb = AABB::from_center(Vec2::new(5.0, 5.0), Vec2::new(2.0, 3.0));
        assert_eq!(aabb.min, Vec2::new(3.0, 2.0));
        assert_eq!(aabb.max, Vec2::new(7.0, 8.0));
    }

    // ── QuadTree tests ──

    #[test]
    fn quadtree_insert_and_len() {
        let bounds = AABB::new(Vec2::new(0.0, 0.0), Vec2::new(100.0, 100.0));
        let mut qt = QuadTree::new(bounds, 4);
        assert_eq!(qt.len(), 0);
        assert!(qt.insert(make_entry(1, 10.0, 10.0)));
        assert!(qt.insert(make_entry(2, 20.0, 20.0)));
        assert_eq!(qt.len(), 2);
    }

    #[test]
    fn quadtree_insert_outside_bounds_fails() {
        let bounds = AABB::new(Vec2::new(0.0, 0.0), Vec2::new(10.0, 10.0));
        let mut qt = QuadTree::new(bounds, 4);
        assert!(!qt.insert(make_entry(1, 20.0, 20.0)));
    }

    #[test]
    fn quadtree_subdivision() {
        let bounds = AABB::new(Vec2::new(0.0, 0.0), Vec2::new(100.0, 100.0));
        let mut qt = QuadTree::new(bounds, 2);
        // Insert 5 entries in the same quadrant to force subdivision
        for i in 0..5 {
            assert!(qt.insert(make_entry(i, 10.0 + i as f64, 10.0)));
        }
        assert_eq!(qt.len(), 5);
        // Children should exist now
        assert!(qt.children.is_some());
    }

    #[test]
    fn quadtree_query_point() {
        let bounds = AABB::new(Vec2::new(0.0, 0.0), Vec2::new(100.0, 100.0));
        let mut qt = QuadTree::new(bounds, 4);
        qt.insert(make_entry(1, 10.0, 10.0));
        qt.insert(make_entry(2, 20.0, 20.0));
        qt.insert(make_entry(3, 10.0, 10.0)); // same pos as entry 1

        let results = qt.query_point(Vec2::new(10.0, 10.0));
        assert_eq!(results.len(), 2);
    }

    #[test]
    fn quadtree_query_range() {
        let bounds = AABB::new(Vec2::new(0.0, 0.0), Vec2::new(100.0, 100.0));
        let mut qt = QuadTree::new(bounds, 4);
        qt.insert(make_entry(1, 10.0, 10.0));
        qt.insert(make_entry(2, 50.0, 50.0));
        qt.insert(make_entry(3, 90.0, 90.0));

        let range = AABB::new(Vec2::new(0.0, 0.0), Vec2::new(55.0, 55.0));
        let results = qt.query_range(range);
        assert!(results.len() >= 2);
        let ids: Vec<u64> = results.iter().map(|e| e.id).collect();
        assert!(ids.contains(&1));
        assert!(ids.contains(&2));
    }

    #[test]
    fn quadtree_query_radius() {
        let bounds = AABB::new(Vec2::new(0.0, 0.0), Vec2::new(100.0, 100.0));
        let mut qt = QuadTree::new(bounds, 4);
        qt.insert(make_entry(1, 10.0, 10.0));
        qt.insert(make_entry(2, 15.0, 10.0)); // 5 units away
        qt.insert(make_entry(3, 80.0, 80.0)); // far away

        let results = qt.query_radius(Vec2::new(10.0, 10.0), 6.0);
        let ids: Vec<u64> = results.iter().map(|e| e.id).collect();
        assert!(ids.contains(&1));
        assert!(ids.contains(&2));
        assert!(!ids.contains(&3));
    }

    #[test]
    fn quadtree_nearest() {
        let bounds = AABB::new(Vec2::new(0.0, 0.0), Vec2::new(100.0, 100.0));
        let mut qt = QuadTree::new(bounds, 4);
        qt.insert(make_entry(1, 10.0, 10.0));
        qt.insert(make_entry(2, 20.0, 20.0));
        qt.insert(make_entry(3, 50.0, 50.0));

        let nearest = qt.nearest(Vec2::new(12.0, 12.0), 2);
        assert_eq!(nearest.len(), 2);
        assert_eq!(nearest[0].0.id, 1); // closest
        assert_eq!(nearest[1].0.id, 2); // second closest
    }

    #[test]
    fn quadtree_clear() {
        let bounds = AABB::new(Vec2::new(0.0, 0.0), Vec2::new(100.0, 100.0));
        let mut qt = QuadTree::new(bounds, 4);
        qt.insert(make_entry(1, 10.0, 10.0));
        qt.insert(make_entry(2, 20.0, 20.0));
        assert_eq!(qt.len(), 2);
        qt.clear();
        assert_eq!(qt.len(), 0);
        assert!(qt.children.is_none());
    }

    // ── GridHash tests ──

    #[test]
    fn gridhash_insert_and_query_cell() {
        let mut gh = GridHash::new(10.0);
        gh.insert(make_entry(1, 5.0, 5.0)); // cell (0, 0)
        gh.insert(make_entry(2, 15.0, 5.0)); // cell (1, 0)
        gh.insert(make_entry(3, 5.0, 15.0)); // cell (0, 1)

        assert_eq!(gh.query_cell((0, 0)).len(), 1);
        assert_eq!(gh.query_cell((1, 0)).len(), 1);
        assert_eq!(gh.query_cell((0, 1)).len(), 1);
        assert_eq!(gh.query_cell((5, 5)).len(), 0);
    }

    #[test]
    fn gridhash_cell_of() {
        let gh = GridHash::new(10.0);
        assert_eq!(gh.cell_of(Vec2::new(5.0, 5.0)), (0, 0));
        assert_eq!(gh.cell_of(Vec2::new(15.0, 5.0)), (1, 0));
        assert_eq!(gh.cell_of(Vec2::new(-5.0, 5.0)), (-1, 0));
    }

    #[test]
    fn gridhash_query_radius() {
        let mut gh = GridHash::new(10.0);
        gh.insert(make_entry(1, 5.0, 5.0));
        gh.insert(make_entry(2, 25.0, 5.0)); // far
        gh.insert(make_entry(3, 8.0, 5.0)); // close to entry 1

        let results = gh.query_radius(Vec2::new(5.0, 5.0), 5.0);
        let ids: Vec<u64> = results.iter().map(|e| e.id).collect();
        assert!(ids.contains(&1));
        assert!(ids.contains(&3));
        assert!(!ids.contains(&2));
    }

    #[test]
    fn gridhash_len() {
        let mut gh = GridHash::new(10.0);
        assert_eq!(gh.len(), 0);
        gh.insert(make_entry(1, 5.0, 5.0));
        gh.insert(make_entry(2, 15.0, 5.0));
        assert_eq!(gh.len(), 2);
    }

    // ── SpatialHash tests ──

    #[test]
    fn spatialhash_insert_large_entry() {
        let mut sh = SpatialHash::new(10.0);
        // Entry spans cells (0,0) through (1,1)
        sh.insert(make_entry_with_size(1, 10.0, 10.0, 8.0));

        assert_eq!(sh.query_cell((0, 0)).len(), 1);
        assert_eq!(sh.query_cell((1, 0)).len(), 1);
        assert_eq!(sh.query_cell((0, 1)).len(), 1);
        assert_eq!(sh.query_cell((1, 1)).len(), 1);
        assert_eq!(sh.query_cell((2, 2)).len(), 0);
    }

    #[test]
    fn spatialhash_query_range() {
        let mut sh = SpatialHash::new(10.0);
        sh.insert(make_entry(1, 5.0, 5.0));
        sh.insert(make_entry(2, 25.0, 5.0));
        sh.insert(make_entry_with_size(3, 15.0, 15.0, 8.0)); // large, overlaps both

        let range = AABB::new(Vec2::new(0.0, 0.0), Vec2::new(12.0, 12.0));
        let results = sh.query_range(range);
        let ids: Vec<u64> = results.iter().map(|e| e.id).collect();
        assert!(ids.contains(&1));
        assert!(ids.contains(&3));
    }

    #[test]
    fn spatialhash_query_radius() {
        let mut sh = SpatialHash::new(10.0);
        sh.insert(make_entry(1, 5.0, 5.0));
        sh.insert(make_entry(2, 25.0, 25.0));

        let results = sh.query_radius(Vec2::new(5.0, 5.0), 3.0);
        let ids: Vec<u64> = results.iter().map(|e| e.id).collect();
        assert!(ids.contains(&1));
        assert!(!ids.contains(&2));
    }

    #[test]
    fn spatialhash_len_dedup() {
        let mut sh = SpatialHash::new(10.0);
        // Large entry goes into multiple cells
        sh.insert(make_entry_with_size(1, 10.0, 10.0, 8.0));
        assert_eq!(sh.len(), 1);
    }

    // ── Cross-structure agreement tests ──

    #[test]
    fn quadtree_and_gridhash_agree_on_radius_query() {
        let bounds = AABB::new(Vec2::new(0.0, 0.0), Vec2::new(200.0, 200.0));
        let mut qt = QuadTree::new(bounds, 8);
        let mut gh = GridHash::new(20.0);

        let entries: Vec<SpatialEntry> = (0..50)
            .map(|i| make_entry(i, (i * 7 % 200) as f64, (i * 13 % 200) as f64))
            .collect();

        for e in &entries {
            qt.insert(e.clone());
            gh.insert(e.clone());
        }

        let center = Vec2::new(100.0, 100.0);
        let radius = 50.0;

        let qt_ids: Vec<u64> = {
            let mut v = qt.query_radius(center, radius).iter().map(|e| e.id).collect::<Vec<_>>();
            v.sort();
            v
        };
        let gh_ids: Vec<u64> = {
            let mut v = gh.query_radius(center, radius).iter().map(|e| e.id).collect::<Vec<_>>();
            v.sort();
            v
        };

        assert_eq!(qt_ids, gh_ids, "QuadTree and GridHash should return the same entries for radius query");
    }

    #[test]
    fn quadtree_and_spatialhash_agree_on_radius_query() {
        let bounds = AABB::new(Vec2::new(0.0, 0.0), Vec2::new(200.0, 200.0));
        let mut qt = QuadTree::new(bounds, 8);
        let mut sh = SpatialHash::new(20.0);

        let entries: Vec<SpatialEntry> = (0..50)
            .map(|i| make_entry(i, (i * 7 % 200) as f64, (i * 13 % 200) as f64))
            .collect();

        for e in &entries {
            qt.insert(e.clone());
            sh.insert(e.clone());
        }

        let center = Vec2::new(100.0, 100.0);
        let radius = 50.0;

        let qt_ids: Vec<u64> = {
            let mut v = qt.query_radius(center, radius).iter().map(|e| e.id).collect::<Vec<_>>();
            v.sort();
            v
        };
        let sh_ids: Vec<u64> = {
            let mut v = sh.query_radius(center, radius).iter().map(|e| e.id).collect::<Vec<_>>();
            v.sort();
            v
        };

        assert_eq!(qt_ids, sh_ids, "QuadTree and SpatialHash should return the same entries for radius query");
    }

    // ── Serde tests ──

    #[test]
    fn vec2_serde_roundtrip() {
        let v = Vec2::new(1.5, -3.7);
        let json = serde_json::to_string(&v).unwrap();
        let v2: Vec2 = serde_json::from_str(&json).unwrap();
        assert_eq!(v, v2);
    }

    #[test]
    fn aabb_serde_roundtrip() {
        let a = AABB::new(Vec2::new(1.0, 2.0), Vec2::new(3.0, 4.0));
        let json = serde_json::to_string(&a).unwrap();
        let a2: AABB = serde_json::from_str(&json).unwrap();
        assert_eq!(a, a2);
    }

    #[test]
    fn entry_serde_roundtrip() {
        let e = make_entry(42, 10.0, 20.0);
        let json = serde_json::to_string(&e).unwrap();
        let e2: SpatialEntry = serde_json::from_str(&json).unwrap();
        assert_eq!(e2.id, 42);
        assert_eq!(e2.pos, Vec2::new(10.0, 20.0));
    }

    // GridHash uses HashMap<(i32,i32),_> which can't roundtrip through JSON
    // (tuple keys aren't JSON strings), but the Serde derives work with
    // binary formats like bincode. Verify derive compiles via the other tests.
}
