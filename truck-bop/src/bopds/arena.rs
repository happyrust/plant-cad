//! Typed arena for indexed storage

/// A typed arena that maps unique IDs to stored values
#[derive(Debug)]
pub struct Arena<T, Id> {
    items: Vec<T>,
    id_from_index: fn(u32) -> Id,
}

impl<T, Id> Arena<T, Id> {
    /// Create a new arena with the given ID constructor
    pub fn new(id_from_index: fn(u32) -> Id) -> Self {
        Self {
            items: Vec::new(),
            id_from_index,
        }
    }

    /// Push a value and return its ID
    pub fn push(&mut self, value: T) -> Id {
        let index = self.items.len() as u32;
        self.items.push(value);
        (self.id_from_index)(index)
    }

    /// Get a reference to the value with the given ID
    pub fn get(&self, id: Id) -> Option<&T>
    where
        Id: Into<u32> + Copy,
    {
        let index: u32 = id.into();
        self.items.get(index as usize)
    }

    /// Get a mutable reference to the value with the given ID
    pub fn get_mut(&mut self, id: Id) -> Option<&mut T>
    where
        Id: Into<u32> + Copy,
    {
        let index: u32 = id.into();
        self.items.get_mut(index as usize)
    }

    /// Get an iterator over the items
    pub fn iter(&self) -> impl Iterator<Item = &T> {
        self.items.iter()
    }

    /// Get the number of items
    pub fn len(&self) -> usize {
        self.items.len()
    }

    /// Check if the arena is empty
    pub fn is_empty(&self) -> bool {
        self.items.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    struct TestId(u32);

    impl From<TestId> for u32 {
        fn from(id: TestId) -> u32 {
            id.0
        }
    }

    #[test]
    fn arena_push_and_get_round_trip() {
        let mut arena = Arena::<&'static str, TestId>::new(TestId);
        let id = arena.push("alpha");
        assert_eq!(arena.get(id), Some(&"alpha"));
    }
}
