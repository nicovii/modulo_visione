# Progetto
 Questo pacchetto è il modulo di visione di un sistema autonomo per la costruzione di un castello composto da quattro tipologie di blocchi di legno.

## Prerequisiti
 Per funzionare, questo modulo, necessita di tre pacchetti:
 * darknet_ros: pacchetto utilizzato per le funzionalità di YOLO.
 * zed_wrapper: pacchetto per integrare la stereo camera ZED con ROS.
 * pcl_action: pacchetto che elabora i dati ricavati dalla stereo camera.
 
 ## Installazione
 Clonare la repository nella cartella /src di un ambiente catkin: 
 ```
 $ git clone https://github.com/nicovii/modulo_visione.git
 ```
 Clonare la cartella darknet in /src dal seguente [sito](https://github.com/leggedrobotics/darknet_ros) e utilizzare i file di configurazione e i pesi precedentemente scaricati.
 
 Efettuare il comando build di catkin, in questo modo:
 ```
 $ catkin build
 ```
 
 ## Modalità di utilizzo
 Per utilizzare questo modulo è necessario aprire tre terminali separati.
 Nel primo avviare il pacchetto zed_wrapper, dopo aver collegato la stereo camera attraverso il cavo USB, in questo modo:
 ```
 $ roslaunch zed_wrapper zed2.launch
 ```
 Nel secondo avviare il software YOLO attraverso il pachhetto darknet_ros, utilizzando questa riga di comando:
 ```
 $ roslaunch darknet_ros darknet_ros.launch
 ```
 Infine nel terzo avviare il pacchetto per l'elaborazione de dati:
 ```
 $ rosrun pcl_action pcl_action
 ```
 ### Funzionamento
 Dopo aver avviato questi tre processi, l'interfaccia con l'utente avvien nel terzo terminale aperto nel quale viene chiesto di posizionare un blocco nell'area predisposta del piano di lavoro e di premere un qualsiasi tasto per efettuare l'analisi.
 Terminati i blocchi da analizzare premere q in questo terminale e chiudere manualmnete gli altri 2.
