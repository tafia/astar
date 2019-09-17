#![feature(test)]
extern crate test;

use astar::{astar, point::Point};
use test::Bencher;

#[bench]
fn corners(b: &mut Bencher) {
    let start = Point::new(0, 0);
    let end = Point::new(64, 64);
    b.iter(|| astar(start, end))
}
