pub struct RtNavi {
    pub should_quit: bool,
}

impl RtNavi {
    pub fn new() -> Self {
        Self { should_quit: false }
    }

    pub fn on_right_key_press(&mut self) {}

    pub fn on_left_key_press(&mut self) {}

    pub fn on_key_press(&mut self, c: char) {
        match c {
            'Q' | 'q' => {
                self.should_quit = true;
            },
            _ => {},
        }
    }
}
