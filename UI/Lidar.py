import ctypes
import traceback
from io import open as openFile
import os
import keyboard
from kivy.app import App
from kivy.config import Config
from kivy.lang import Builder
from kivy.uix.settings import SettingsWithSidebar
import UI

#Config.set('input', 'mouse', 'mouse,multitouch_on_demand')
#Config.set('kivy', 'window_icon', 'assets/logoDark.ico')

# https://stackoverflow.com/questions/34468909/how-to-make-tooltip-using-kivy

Builder.load_file('.\\main.kv')


class LidarUIApp(App):
    def build(self):
        self.settings_cls = SettingsWithSidebar
        #self.icon = 'assets/logoDark.png'
        self.app = UI.UI()

        os.chdir(os.path.dirname(os.path.realpath(__file__)))

        self.configFiles = {
            ".\\config.json": 'General'
        }

        return self.app

    def build_config(self, config):
        config.setdefaults('general', {
            'speed': '1'})

    def build_settings(self, settings):
        for files in self.configFiles:
            f = openFile(files, "r", encoding='utf8')
            if f.mode == 'r':
                contents = f.read()
                settings.add_json_panel(
                    self.configFiles[files], self.config, data=contents)


if __name__ == '__main__':
    LidarUIApp().run()
