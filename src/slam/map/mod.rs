pub mod keyframe;
pub mod mappoint;

use std::collections::HashMap;
use std::rc::Rc;
use std::cell::RefCell;
use std::sync::atomic::{Ordering, AtomicUsize};

use keyframe::*;
use mappoint::*;

static KEYFRAME_ID_COUNTER: AtomicUsize = AtomicUsize::new(0);
const MAX_ID: usize = usize::MAX / 2;

fn generate_id() -> usize{
    // 检查两次溢出，否则直接加一可能导致溢出
    let current_val = KEYFRAME_ID_COUNTER.load(Ordering::Relaxed);
    if current_val > MAX_ID{
        panic!("Factory ids overflowed");
    }
    let next_id = KEYFRAME_ID_COUNTER.fetch_add(1, Ordering::Relaxed);
    if next_id > MAX_ID{
        panic!("Factory ids overflowed");
    }
    next_id
}


#[derive(Clone)]
pub struct Map {
    pub keyframes: HashMap<KeyFrameId, KeyFrame>,
    pub mappoints: HashMap<MapPointId, Rc<RefCell<MapPoint>>>,
}

impl Map {
    pub fn new() -> Self {
        Self {
            keyframes: HashMap::new(),
            mappoints: HashMap::new(),
        }
    }

    pub fn insert_keyframe(&mut self, keyframe: KeyFrame) {
        self.keyframes.insert(keyframe.id, keyframe);
    }

    pub fn insert_mappoint(&mut self, mappoint: Rc<RefCell<MapPoint>>) {
        let id = mappoint.borrow().id;
        self.mappoints.insert(id, mappoint);
    }

    pub fn next_id(&self) -> KeyFrameId {
        generate_id()
    }

    pub fn points(&self) -> Vec<Rc<RefCell<MapPoint>>> {
        self.mappoints.values().cloned().collect()
    }
}