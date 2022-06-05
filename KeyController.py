from pynput.keyboard import Listener, KeyCode


class KeyController:
    def __init__(self):
        self.listener = Listener(
            on_press=self.on_press,
            on_release=self.on_release)
        self.key_pressed = []
        self.run()

    def on_press(self, key):
        # print('{0} pressed'.format(key))
        if key not in self.key_pressed:
            self.key_pressed.append(key)

    def on_release(self, key):
        # print('{0} released'.format(key))
        if key in self.key_pressed:
            self.key_pressed.remove(key)
        if key == KeyCode.from_char('t'):
            self.listener.stop()
            return False

    def run(self):
        self.listener.start()

    def stop(self):
        self.listener.stop()

    def get_key_pressed(self) -> list:
        return self.key_pressed


if __name__ == '__main__':
    kc = KeyController()
    while kc.listener.running:
        if kc.get_key_pressed() != []:
            print(kc.key_pressed)
