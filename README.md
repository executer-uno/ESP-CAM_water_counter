# ESP-CAM_water_counter
ESP-CAM recogize values from mechanical dispaly water counter, stores to GSpreadsheets

Great thanks for NickVectra, author of original project:
http://forum.arduino.ua/viewtopic.php?pid=31135#p31135

Most complicate thing of text recognition will be taken from his project.

Actual browser output view looks like:
![Actual](https://github.com/executer-uno/ESP-CAM_water_counter/blob/Master/2020-05-11_171527.png)

Parameters page:
![Parameters](https://github.com/executer-uno/ESP-CAM_water_counter/blob/Master/2020-05-11_171538.png)


How to get that repo into Sloeber IDE:
•	Sloeber IDE installed ( http://eclipse.baeyens.it/stable.php?OS=Windows )
•	Git installed ( https://git-scm.com/download/win )
I found no way to get github project locally by Sloeber interface, so need to help it a bit. Go to sloeber repos folder and create there a new one for our new project (from git terminal):
$ cd c:/Users/E_CAD/git/
$ mkdir ESP_CAM_GIT
$ cd ESP_CAM_GIT
$ git clone https://github.com/executer-uno/ESP-CAM_water_counter.git

Now we got our repo locally and the Sloeber project is in repo’s subfolder:
C:\Users\E_CAD\git\ESP_CAM_GIT\ESP-CAM_water_counter\ESP_Cam_Counter
In Sloeber create new Arduino project “ESP_Cam_Counter” and set location as
C:\Users\E_CAD\git\ESP_CAM_GIT\ESP-CAM_water_counter\ESP_Cam_Counter
Setup board configuration and on next dialog keeps “default ino file”. It will blames “failed to create project”. Press “OK”.
Project with connected repo is ready in Sloeber.
