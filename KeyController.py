from threading import Thread

from pynput.keyboard import Listener, KeyCode


class KeyController:
    def __init__(self):
        self.listener = Listener(
            on_press=self.on_press,
            on_release=self.on_release)
        self.key_pressed = []
        # self.thread = Thread(target=self.run, args=())
        # self.thread.start()
        self.run()

    def on_press(self, key: KeyCode):
        # print('{0} release'.format(key))
        keychar = str(key).replace("\'", "").lower()
        if keychar not in self.key_pressed:
            self.key_pressed.append(keychar)

    def on_release(self, key: KeyCode):
        # print('{0} release'.format(key))
        keychar = str(key).replace("\'", "").lower()
        if keychar in self.key_pressed:
            self.key_pressed.remove(keychar)
        if key == KeyCode.from_char('t'):
            # Stop listenerd
            return False

    def run(self):
        # with self.listener:
        #     self.listener.join()
        # self.stop()
        self.listener.start()

    def stop(self):
        self.listener.stop()

    def get_key_pressed(self) -> list:
        return self.key_pressed


if __name__ == '__main__':
    kc = KeyController()
    while kc.listener.running:
        if kc.get_key_pressed() != []:
            print(kc.get_key_pressed())
