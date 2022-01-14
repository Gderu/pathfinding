use show_image::{ImageView, ImageInfo, create_window, WindowProxy};
use rand::Rng;
use rand::prelude::ThreadRng;
use priority_queue::PriorityQueue;
use std::cmp::{Reverse, Ordering};
use std::rc::Rc;
use std::hash::{Hash, Hasher};
use std::time::Instant;
use std::collections::HashSet;

const MAP_WIDTH: usize = 500;
const MAP_HEIGHT: usize = 350;
const WALL_LENGTH: usize = 50;
const PIXEL_SIZE: usize = 3;
const NUM_WALLS: usize = 400;

#[derive(Eq, Debug, Clone)]
struct Node {
    state: (usize, usize),
    parent: Option<Rc<Node>>,
    cost: usize,
}

impl PartialEq for Node {
    fn eq(&self, other: &Self) -> bool {
        self.state == other.state
    }
}

impl Hash for Node {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.state.hash(state);
    }
}

#[derive(Eq, PartialEq, Hash, Debug, Ord)]
struct PairOrder {
    a: usize,
    b: usize,
}

impl PartialOrd for PairOrder {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(if self.a > other.a {
            Ordering::Greater
        } else if self.a < other.a {
            Ordering::Less
        } else if self.b > other.b {
            Ordering::Greater
        } else if self.b < other.b {
            Ordering::Less
        } else {
            Ordering::Equal
        })
    }
}

#[show_image::main]
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut rng = rand::thread_rng();
    let map = create_map(&mut rng);
    let start_pos = (rng.gen_range(0..MAP_WIDTH), rng.gen_range(0..MAP_HEIGHT));
    let goal_pos = (rng.gen_range(0..MAP_WIDTH), rng.gen_range(0..MAP_HEIGHT));
    create_window_map("A*", map.clone(), start_pos, goal_pos, |n| a_star_f(n, goal_pos))?;
    let window = create_window_map("Weighted A*", map, start_pos, goal_pos, |n| weighted_a_star_f(n, goal_pos));
    if window.is_ok() {
        window.unwrap().wait_until_destroyed()?;
    }
    Ok(())
}

fn create_window_map(name: &str, mut map: [u8; MAP_WIDTH * MAP_HEIGHT * PIXEL_SIZE], start_pos: (usize, usize), goal_pos: (usize, usize), f: impl Fn(&Node) -> usize) -> Result<WindowProxy, Box<dyn std::error::Error>> {
    let now = Instant::now();
    println!("{}", name);
    tree_like_search(&mut map, start_pos, goal_pos, f);
    println!("Elapsed: {:.2?}", now.elapsed());
    let image = ImageView::new(ImageInfo::rgb8(MAP_WIDTH as u32, MAP_HEIGHT as u32), &map);
    let window = create_window(name, Default::default())?;
    window.set_image(name, image)?;
    Ok(window)

}

fn a_star_f(n: &Node, goal_pos: (usize, usize)) -> usize {
    n.cost + calc_distance(n.state, goal_pos)
}

fn weighted_a_star_f(n: &Node, goal_pos: (usize, usize)) -> usize {
    n.cost + 2 * calc_distance(n.state, goal_pos)
}

fn calc_distance(a: (usize, usize), b: (usize, usize)) -> usize {
    let x_dist = if a.0 > b.0 {
        a.0 - b.0
    } else {
        b.0 - a.0
    };
    let y_dist = if a.1 > b.1 {
        a.1 - b.1
    } else {
        b.1 - a.1
    };
    x_dist + y_dist
}

fn create_map(rng: &mut ThreadRng) -> [u8; MAP_WIDTH * MAP_HEIGHT * PIXEL_SIZE] {
    let mut map = [255; MAP_WIDTH * MAP_HEIGHT * PIXEL_SIZE];
    for _ in 0..(NUM_WALLS / 2) {
        let x = rng.gen_range(0..MAP_WIDTH);
        let y = rng.gen_range(0..(MAP_HEIGHT - WALL_LENGTH));
        (y..(y + WALL_LENGTH)).for_each(|y| {
            map[(y * MAP_WIDTH + x) * PIXEL_SIZE] = 0;
            map[(y * MAP_WIDTH + x) * PIXEL_SIZE + 1] = 0;
            map[(y * MAP_WIDTH + x) * PIXEL_SIZE + 2] = 0;
        });
    }
    for _ in 0..(NUM_WALLS / 2) {
        let y = rng.gen_range(0..MAP_HEIGHT);
        let x = rng.gen_range(0..(MAP_WIDTH - WALL_LENGTH));
        (x..(x + WALL_LENGTH)).for_each(|x| {
            map[(y * MAP_WIDTH + x) * PIXEL_SIZE] = 0;
            map[(y * MAP_WIDTH + x) * PIXEL_SIZE + 1] = 0;
            map[(y * MAP_WIDTH + x) * PIXEL_SIZE + 2] = 0;
        });
    }
    map
}

fn tree_like_search(map: &mut [u8; MAP_WIDTH * MAP_HEIGHT * PIXEL_SIZE], start_pos: (usize, usize), goal_pos: (usize, usize), f: impl Fn(&Node) -> usize) {
    let mut frontier: PriorityQueue<Rc<Node>, Reverse<PairOrder>> = PriorityQueue::new();
    let mut node = Rc::new(Node {state: start_pos, parent: None, cost: 0});
    let mut found: HashSet<Rc<Node>> = HashSet::new();
    found.insert(Rc::clone(&node));
    while node.state != goal_pos {
        for action in [(0, 1), (0, -1), (-1, 0), (1, 0)] {
            if let Some(to_add) = add_pos(node.state, action) {
                if map[(to_add.1 * MAP_WIDTH + to_add.0) * PIXEL_SIZE] != 0 {
                    map[(to_add.1 * MAP_WIDTH + to_add.0) * PIXEL_SIZE] = 128;
                    map[(to_add.1 * MAP_WIDTH + to_add.0) * PIXEL_SIZE + 1] = 128;
                    map[(to_add.1 * MAP_WIDTH + to_add.0) * PIXEL_SIZE + 2] = 255;
                    let new_node = Rc::new(Node {state: to_add, parent: Some(Rc::clone(&node)), cost: node.cost + 1});
                    let f_n = f(&new_node);
                    let priority = Reverse(PairOrder { a: f_n, b: f_n - new_node.cost });
                    if found.get(&new_node).is_none() {
                        frontier.push(Rc::clone(&new_node), priority);
                        found.insert(Rc::clone(&new_node));
                    }
                }
            }
        }
        node = match frontier.pop() {
            Some((n, _p)) => n,
            None => {
                println!("No solution exists");
                draw_start_goal(start_pos, goal_pos, map);
                return
            },
        };
    }
    let mut node = &node;
    let mut count = 0;
    while node.parent.is_some() {
        count += 1;
        map[(node.state.1 * MAP_WIDTH + node.state.0) * PIXEL_SIZE] = 255;
        map[(node.state.1 * MAP_WIDTH + node.state.0) * PIXEL_SIZE + 1] = 0;
        map[(node.state.1 * MAP_WIDTH + node.state.0) * PIXEL_SIZE + 2] = 0;
        node = &node.parent.as_ref().unwrap();
    }
    draw_start_goal(start_pos, goal_pos, map);
    println!("Path length: {}", count);
}

fn draw_start_goal(start_pos: (usize, usize), goal_pos: (usize, usize), map: &mut [u8; MAP_WIDTH * MAP_HEIGHT * PIXEL_SIZE]) {
    map[(goal_pos.1 * MAP_WIDTH + goal_pos.0) * PIXEL_SIZE] = 0;
    map[(goal_pos.1 * MAP_WIDTH + goal_pos.0) * PIXEL_SIZE + 1] = 255;
    map[(goal_pos.1 * MAP_WIDTH + goal_pos.0) * PIXEL_SIZE + 2] = 0;
    map[(start_pos.1 * MAP_WIDTH + start_pos.0) * PIXEL_SIZE] = 0;
    map[(start_pos.1 * MAP_WIDTH + start_pos.0) * PIXEL_SIZE + 1] = 0;
    map[(start_pos.1 * MAP_WIDTH + start_pos.0) * PIXEL_SIZE + 2] = 255;
}

fn add_pos(a: (usize, usize), b: (isize, isize)) -> Option<(usize, usize)> {
    let x = if b.0.is_negative() {
        a.0.checked_sub(b.0.abs() as usize)
    } else {
        a.0.checked_add(b.0 as usize)
    };
    let y = if b.1.is_negative() {
        a.1.checked_sub(b.1.abs() as usize)
    } else {
        a.1.checked_add(b.1 as usize)
    };
    x.filter(|&v| v < MAP_WIDTH).zip(y.filter(|&v| v < MAP_HEIGHT))
}