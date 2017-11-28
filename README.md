# Study Companion Robot

### How does a robot that studies with you affect your concentration and productivity? :sunglasses:

The idea of a study companion robot stems from the question above. How is it like to have a robotic study buddy? In order to test out the hypothesis behind the question, we designed a socially assistive robot that can help students engaged in a writing task.



## Setting up

### Keylogger

`cd` to the project directory and launch the keylogger:

```bash
$ sudo keylogger
```

### Applescript

Open [`applescript/windows_monitor.scpt`](https://github.com/kelvinhu9988/study-companion-robot/blob/master/applescript/windows_monitor.scpt#L12) using Mac OS X's Script Editor, and modify the file path to match the current project directory.


### Web Server
Modify the port number in [`web/server.py`](https://github.com/kelvinhu9988/study-companion-robot/blob/master/web/server.py#L12), [`web/index.html`](https://github.com/kelvinhu9988/study-companion-robot/blob/master/web/index.html#L12), and [`monitor.py`](https://github.com/kelvinhu9988/study-companion-robot/blob/master/monitor.py#L12) to the desired port number that you want to use. Then run the server using the following script:

```bash
$ sudo python /projectPath/web/server.py
```

### Facial Expression Monitor

After running the server, open a browser (preferably the latest version of Chrome) and open the following URL: `http://localhost:8899/index.html` to start monitoring the facial expression. Change `8899` to the correct port number you set in the previous step.

