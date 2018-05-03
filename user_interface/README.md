# Spyndra Remote User Interface

## User Interface
1. Start the UI by running, this requires the corresponding server.py running at spyndra
```
$ python ui.py
```

2. Enter the socket address, port number in the format like (127.0.0.1, 80) and click connect. 
A prompt will show if the connection succeeded or not.

3. Click the command button and the command code will be sent to the remote server

## Remote Server
1. the remote server has to be running at startup of spyndra

remaining issues:
1. there is a bad descriptor problem when user node publishes command code through channel /user_cmd 