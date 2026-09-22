# Distanze dai limiti articolari con rqt_plot

Patch per `zanellar/franka_ws`, branch `franka_ws_013_gazebo`, commit
`5e1c727c7bb4725c1ef2a4b0604f720f3ddaf647` (stato letto da GitHub il 21/09/2026).
Workspace locale: `franka_ws_013`.

## Cosa aggiunge

- `scripts/joint_limit_diagnostics.py`: legge i limiti di posizione dell'URDF
  caricato, riceve JointState e pubblica i margini con segno.
- `scripts/plot_joint_limits.py`: aspetta i publisher e un primo campione,
  poi avvia rqt_plot con le curve corrette per quell'URDF e una linea a zero.
- `launch/joint_limit_diagnostics.launch`: avvio autonomo della funzionalita.
- Inclusione nel launch Gazebo del controllore CBF direzionale.
- Installazione dei due script via `catkin_install_python` e dipendenze runtime.
- Nove test offline.

Nessuna modifica ai file C++, ai messaggi ROS custom, alla legge di controllo,
alla dinamica di Gazebo o al comportamento del recorder CSV.
La diagnostica funziona prima della prima chiamata al servizio esperimento
ed e indipendente da `cbf_active` e `record_cbf`.

## Applicare la patch

Questa patch si applica direttamente al commit indicato. Non serve riapplicare
le precedenti patch per CSV e plot, gia presenti in quel commit.
Estrai lo ZIP. Sostituisci `/percorso/joint_limit_diagnostics` con la cartella
estratta e `/percorso/franka_ws_013` con il tuo workspace:

```bash
cd /percorso/franka_ws_013
git apply --check /percorso/joint_limit_diagnostics/joint_limit_diagnostics.patch
git apply /percorso/joint_limit_diagnostics/joint_limit_diagnostics.patch
```

Esegui il secondo comando solo se il primo termina senza errori.
Non copiare prima i file da `files/`: sono inclusi nella patch e forniti
separatamente soltanto per consultazione. In presenza di modifiche locali
sugli stessi punti, risolvi il conflitto segnalato da `--check` prima di applicare.

## Dipendenze e installazione

Con ROS Noetic, se rqt_plot o i messaggi standard non sono gia installati:

```bash
sudo apt install ros-noetic-rqt-plot ros-noetic-rospy \
  ros-noetic-sensor-msgs ros-noetic-std-msgs ros-noetic-rosbash
```

Da `franka_ws_013`, usa la tua configurazione di build:

```bash
export CC=/usr/bin/gcc-10
export CXX=/usr/bin/g++-10
export FRANKA_013_PREFIX="$HOME/Riccardo/libfranka-0.13.3/install"

catkin_make install \
  -DCMAKE_C_COMPILER=/usr/bin/gcc-10 \
  -DCMAKE_CXX_COMPILER=/usr/bin/g++-10 \
  -DCMAKE_CXX_STANDARD=17 \
  -DCMAKE_CXX_STANDARD_REQUIRED=ON \
  -DCMAKE_CXX_EXTENSIONS=OFF \
  -DABSL_PROPAGATE_CXX_STD=ON \
  -DABSL_BUILD_TESTING=OFF \
  -DOSQP-CPP_BUILD_TESTS=OFF \
  -DCMAKE_PREFIX_PATH="$FRANKA_013_PREFIX;/opt/ros/noetic" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCATKIN_ENABLE_TESTING=OFF
```

Mantieni il percorso libfranka gia funzionante sulla tua macchina. Solo dopo
una build riuscita:

```bash
cp -a devel/lib/libabsl*.so* install/lib/
source install/setup.bash
```

La build serve a installare gli script e i launch nuovi: non cambia alcuna
interfaccia C++ o definizione di messaggio. Non occorre compilare manualmente Python.
Esegui `source /percorso/franka_ws_013/install/setup.bash` nei terminali ROS usati.

## Avvio insieme all'esperimento

```bash
roslaunch franka_example_controllers \
  cartesian_impedance_directional_kinetic_energy_cbf_controller_gazebo.launch \
  start_trajectory:=true trajectory:=linear \
  joint_limit_diagnostics:=true plot_joint_limits:=true
```

Puoi aggiungere gli altri argomenti del tuo esperimento, inclusi `log_dir`,
`Kmax`, `alpha`, direzione e `rviz`. I comandi del servizio restano invariati.
Attendi che la finestra sia aperta prima di avviare un esperimento che vuoi
osservare dall'inizio. Se Gazebo parte in pausa, la finestra aspetta il primo
messaggio di stato; sblocca la simulazione per far arrivare i dati.

| Argomento | Default | Effetto |
| --- | --- | --- |
| `joint_limit_diagnostics` | `true` | Avvia il nodo dei margini |
| `plot_joint_limits` | `false` | Se true, apre rqt_plot con le curve e attiva anche la diagnostica |

Per disattivare tutto passa entrambi a `false`. Per usare la diagnostica senza
GUI lascia `plot_joint_limits:=false`. Una richiesta di plot abilita sempre
anche la sua sorgente dati, persino con `joint_limit_diagnostics:=false`.
L'opzione `headless` del launch Gazebo non modifica `plot_joint_limits`: la GUI
richiede un ambiente grafico disponibile. Chiudere rqt_plot non ferma gli altri
nodi e la finestra non viene riaperta automaticamente.

## Avvio su una simulazione gia in esecuzione

Se la diagnostica NON e gia avviata dal launch principale:

```bash
roslaunch franka_example_controllers joint_limit_diagnostics.launch \
  arm_id:=fr3 plot_joint_limits:=true
```

Questo launch non avvia un secondo Gazebo e non modifica i controllori.
Non avviarlo insieme a una diagnostica gia in esecuzione con lo stesso nome.
Se il nodo diagnostico e gia attivo e vuoi aprire/riaprire solo la finestra:

```bash
rosrun franka_example_controllers plot_joint_limits.py
```

Per usare un namespace diverso, il solo avviatore della GUI accetta
`_diagnostics_node:=/namespace/joint_limit_diagnostics`.
Il launch autonomo accetta anche `joint_states_topic` e `robot_description_param`.

## Topic e significato

Input default: `/franka_state_controller/joint_states`. Il nodo usa `name` per
associare ogni posizione al suo giunto, senza assumere un ordine degli array.
I finger joint vengono ignorati: per default sono richiesti i sette
`<arm_id>_joint1` ... `<arm_id>_joint7`.

Limiti: attributi `lower` e `upper` di `<joint><limit>` in `/robot_description`.
Sono i limiti di posizione URDF, non le `soft_lower_limit`/`soft_upper_limit`.
Vengono letti all'avvio: se cambi URDF, riavvia diagnostica e finestra.

Per limiti simmetrici `[-L,+L]`:

```
margin = L - abs(q)
```

Per limiti asimmetrici:

```
lower_margin = q - lower
upper_margin = upper - q
```

Positivo: dentro il limite. Zero: limite raggiunto. Negativo: limite superato.
Non viene applicato un valore assoluto al margine. La simmetria e rilevata
con tolleranza assoluta di `1e-9 rad` (parametro privato `symmetry_tolerance`).
Anche quando la simmetria e approssimata, il margine usa i limiti effettivi:
`min(q-lower, upper-q)`, equivalente a `L-abs(q)` nel caso esattamente simmetrico.

Tutti i topic numerici sono `std_msgs/Float64`, in radianti.
Nell'URDF FR3 della sessione fornita si ottengono queste 9 curve:

| Campo per rqt_plot, dopo `/joint_limit_diagnostics/` | Grandezza |
| --- | --- |
| `fr3_joint1/margin/data` | Margine giunto 1 |
| `fr3_joint2/margin/data` | Margine giunto 2 |
| `fr3_joint3/margin/data` | Margine giunto 3 |
| `fr3_joint4/lower_margin/data` | Distanza dal limite inferiore giunto 4 |
| `fr3_joint4/upper_margin/data` | Distanza dal limite superiore giunto 4 |
| `fr3_joint5/margin/data` | Margine giunto 5 |
| `fr3_joint6/lower_margin/data` | Distanza dal limite inferiore giunto 6 |
| `fr3_joint6/upper_margin/data` | Distanza dal limite superiore giunto 6 |
| `fr3_joint7/margin/data` | Margine giunto 7 |
| `zero/data` | Linea di riferimento zero (decima curva) |

I nomi e il numero di curve vengono derivati dall'URDF, non codificati per FR3.
Configurazione pubblicata come JSON in un messaggio String latched:

```bash
rostopic echo -n 1 /joint_limit_diagnostics/configuration
```

Questa configurazione elenca limiti e topic; l'avviatore la usa per configurare
rqt_plot automaticamente. Aspetta anche un messaggio `zero` prima di aprire
la GUI. Passa `--empty` a rqt_plot per non ripristinare curve di vecchie sessioni.
Selezione dei topic automatica; zoom, colori e assi restano quelli gestiti da rqt_plot.

Per osservare solo il giunto 6, puoi aprire manualmente:

```bash
rosrun rqt_plot rqt_plot --empty \
  /joint_limit_diagnostics/fr3_joint6/lower_margin/data \
  /joint_limit_diagnostics/fr3_joint6/upper_margin/data \
  /joint_limit_diagnostics/zero/data
```

## Frequenza, dati mancanti e interpretazione

Il nodo calcola e pubblica a ogni JointState ricevuto, senza timer di
ricampionamento o decimazione. Nel launch attuale `state_publish_rate=1000`,
quindi la frequenza nominale di ingresso e 1 kHz di tempo simulato. Non usa il
`/joint_states` ripubblicato a 30 Hz. Puoi controllare il flusso con:

```bash
rostopic hz /franka_state_controller/joint_states
rostopic hz /joint_limit_diagnostics/fr3_joint6/lower_margin
```

Il nodo pubblica NaN per giunti mancanti o non finiti. Array di nomi/posizioni
incoerenti o nomi duplicati invalidano l'intero messaggio. Non riutilizza
posizioni precedenti e non pubblica zeri come sostituto dei dati mancanti.
Se lo stream si ferma, non genera nuovi campioni artificiali.

I messaggi Float64 non hanno header: rqt_plot usa il tempo ROS di ricezione,
non il timestamp originale di JointState. Questi topic servono per osservare
i margini dal vivo, non per misurare sincronismi al millisecondo con la CBF.
La frequenza effettiva dipende dalla simulazione e dal carico; le code ROS e
la GUI possono perdere campioni. Per una diagnosi temporale usa il CSV gia
previsto, che conserva q e CBF nello stesso campione quando la CBF e attiva.

Una curva negativa mostra che la posizione riportata ha oltrepassato il
limite URDF. Da sola non misura la forza del fine corsa e non dimostra la causa
di una violazione energetica. Non vengono rimossi o modificati i fine corsa.

## Verifiche

Eseguiti nove test offline: formule e segni, estremi e superamenti, mappatura
per nome, giunti extra, dati mancanti/non finiti/duplicati, URDF invalido,
simmetria, pubblicazione, configurazione e passaggio di avvio a rqt_plot,
condizioni del launch. Verificati anche i nove topic con l'URDF della sessione
fornita e l'applicazione della patch sui file esatti del commit indicato.

Per ripetere i test nel workspace:

```bash
python3 src/franka_ros/franka_example_controllers/test/test_joint_limit_diagnostics.py
```

ROS Noetic/Gazebo e una GUI Qt non sono disponibili nell'ambiente di
preparazione: build catkin, trasporto ROS e apertura effettiva di rqt_plot
restano da verificare sulla tua macchina. Nessun test dichiara una misura
reale di prestazioni a 1 kHz.

Riferimento per gli argomenti CLI `--empty` e topic di rqt_plot:
https://github.com/ros-visualization/rqt_plot/blob/noetic-devel/src/rqt_plot/plot.py
