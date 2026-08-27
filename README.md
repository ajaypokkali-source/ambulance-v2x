\# 🚑 Ambulance V2X Simulation



A SUMO- and Python-based simulation project for studying emergency-vehicle communication and traffic response using \*\*V2V\*\* and \*\*V2I\*\* concepts, with \*\*I2I and broader V2X coordination planned for future development\*\*.



The project uses \*\*Eclipse SUMO\*\*, \*\*TraCI\*\*, Python, and XML-based road-network/traffic definitions to simulate an ambulance moving through a traffic environment.



\---



\## 📌 Project Overview



The main objective of this project is to investigate how connected vehicles and intelligent transportation infrastructure can respond to an approaching emergency vehicle.



The current project contains working experiments for:



\- 🚗 \*\*V2V — Vehicle-to-Vehicle\*\*

\- 🚦 \*\*V2I — Vehicle-to-Infrastructure\*\*



Additional work toward:



\- 🔄 \*\*I2I — Infrastructure-to-Infrastructure\*\*

\- 🌐 \*\*V2X — Vehicle-to-Everything\*\*



is planned as the project develops.



The ambulance is used as the emergency vehicle, while surrounding vehicles and traffic infrastructure respond according to the communication logic implemented in the Python controllers.



\---



\# 🧩 Technologies



\- \*\*Eclipse SUMO 1.27.1\*\*

\- \*\*Python 3\*\*

\- \*\*TraCI\*\*

\- \*\*sumolib\*\*

\- \*\*XML\*\*

\- \*\*Git\*\*

\- \*\*GitHub\*\*



\---



\# 📁 Project Structure



```text

ambulance\_v2x/

│

├── python/

│   └── run\_simulation.py

│

├── simulation/

│   ├── edges.edg.xml

│   ├── network.net.xml

│   ├── nodes.nod.xml

│   ├── routes.rou.xml

│   ├── simulation.sumocfg

│   ├── traffic\_light\_test.py

│   └── v2x\_corridor\_v1.py

│

├── tls\_simulation/

│   ├── edges.edg.xml

│   ├── network.net.xml

│   ├── nodes.nod.xml

│   ├── routes.rou.xml

│   ├── simulation.sumocfg

│   ├── traffic\_light\_test.py

│   ├── i2i\_corridor\_test.py

│   ├── v2i\_i2i\_test.py

│   └── v2i\_signal\_test.py

│

├── v2v\_stress\_test/

│   ├── edges.edg.xml

│   ├── network.net.xml

│   ├── nodes.nod.xml

│   ├── routes.rou.xml

│   ├── simulation.sumocfg

│   ├── test.rou.xml

│   ├── test.rou.alt.xml

│   └── v2v\_stress\_test.py

│

├── .gitignore

└── README.md

