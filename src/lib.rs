use std::collections::{BinaryHeap, VecDeque};

type HashMap<K, V> = fxhash::FxHashMap<K, V>;

pub mod point {
    use super::AStarPoint;

    #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
    pub struct Point {
        pub x: i32,
        pub y: i32,
    }

    fn absdiff(a: i32, b: i32) -> u32 {
        if a > b {
            (a - b) as u32
        } else {
            (b - a) as u32
        }
    }

    impl Point {
        pub fn new(x: i32, y: i32) -> Self {
            Point { x, y }
        }
    }

    impl AStarPoint for Point {
        const MOVE_COST: u32 = 1;
        type Neighbors = Vec<Point>;

        fn heuristic(&self, end: &Self) -> u32 {
            absdiff(self.x, end.x) + absdiff(self.y, end.y)
        }

        fn neighbors(&self) -> Self::Neighbors {
            let Point { x, y } = *self;

            vec![
                Point::new(x, y + 1),
                Point::new(x, y - 1),
                Point::new(x + 1, y),
                Point::new(x - 1, y),
                //Point::new(self.x - 1, self.y + 1),
                //Point::new(self.x - 1, self.y - 1),
                //Point::new(self.x + 1, self.y + 1),
                //Point::new(self.x + 1, self.y - 1),
            ]
        }
    }
}

#[derive(Debug)]
struct Node<P> {
    point: P,
    source: Option<P>,
    f: u32,
    g: u32,
}

impl<P: PartialEq> PartialEq for Node<P> {
    fn eq(&self, other: &Self) -> bool {
        self.point.eq(&other.point) && self.f.eq(&other.f)
    }
}

impl<P: PartialEq> Eq for Node<P> {}

impl<P: Ord> PartialOrd for Node<P> {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl<P: Ord> Ord for Node<P> {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.f
            .cmp(&other.f)
            .reverse()
            .then(self.point.cmp(&other.point))
    }
}

pub trait AStarPoint: Eq + std::hash::Hash + Ord + Copy {
    const MOVE_COST: u32;
    type Neighbors: IntoIterator<Item = Self>;
    fn neighbors(&self) -> Self::Neighbors;
    fn heuristic(&self, end: &Self) -> u32;
}

pub fn astar<P: AStarPoint>(a: P, b: P) -> VecDeque<P> {
    let mut opened = BinaryHeap::new();
    opened.push(Node {
        point: a,
        source: None,
        f: a.heuristic(&b),
        g: 0,
    });
    let mut costs = HashMap::default();
    let mut closed = HashMap::default();

    while let Some(node) = opened.pop() {
        if node.point == b {
            closed.insert(node.point, node);
            break;
        }
        opened.extend(
            node.point
                .neighbors()
                .into_iter()
                .filter(|n| !closed.contains_key(n))
                .filter_map(|n| {
                    let g = node.g + P::MOVE_COST;
                    let h = n.heuristic(&b);
                    let f = g + h;
                    let prev_cost = costs.entry(n).or_insert(std::u32::MAX);
                    if *prev_cost <= f {
                        None
                    } else {
                        *prev_cost = f;
                        Some(Node {
                            point: n,
                            source: Some(node.point),
                            f,
                            g,
                        })
                    }
                }),
        );
        closed.insert(node.point, node);
    }

    // generate path starting from b
    let mut path = VecDeque::with_capacity(closed.len());
    let mut last = b;
    while let Some(node) = closed.get_mut(&last) {
        path.push_front(last);
        if let Some(source) = node.source.take() {
            last = source;
        } else {
            break;
        }
    }
    path
}

#[test]
fn diag() {
    let a = point::Point::new(0, 0);
    let b = point::Point::new(3, 2);
    let path = astar(a, b);
    assert_eq!(path.len(), 6);
    assert_eq!(path.front(), Some(&a));
    assert_eq!(path.back(), Some(&b));
}
