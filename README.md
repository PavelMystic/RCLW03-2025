
Efficient sensor scheduling is a critical challenge in multi-functional passive surveillance systems, where hardware resources must be optimally allocated to balance multiple objectives, such as electromagnetic spectrum survey and target tracking. This paper frames the sensor scheduling problem within the Optimal Experimental Design (OED) paradigm, leveraging information-theoretic and Bayesian optimization principles. We formulate the scheduling task as a multi-objective optimization problem, where one objective aims to maximize information gain in target tracking using Rényi divergence, while the other seeks to ensure an unbiased survey of the frequency spectrum through Kullback-Leibler divergence minimization. To effectively handle the trade-offs between these conflicting goals, we apply goal programming scalarization, offering a more robust alternative to traditional epsilon-constraint methods. A simulation-based case study demonstrates how the proposed optimization framework dynamically adjusts sensor tuning strategies, achieving a balanced allocation of system resources. The results highlight the potential of experimental design methodologies for improving real-time sensor tasking in complex, uncertain environments. This work bridges the gap between decision-theoretic experimental desig and practical sensor scheduling applications, providing insights applicable to adaptive sensing, electronic warfare, and autonomous surveillance systems.

# Results

As a result of running our algorithm on simulated data, we have obtained tuning plan that is visualized. Tuning in each of the receving channel is distinguished by color. The cases when all four channels were tuned into the same frequency band are emphasized using red bounding box. One can see that these four channel tunings are prevalent in bands 1,4,7,9 as this is caused by the tracking objective in order to gain track update for the targets emitting there.

<img src="img/tuning.png" alt="Tuning plan"/>

Above you can see resulting tuning plan for 10 frequency bands, 4 receivers, 4 targets in bands 1,4,7,9, random stationary distribution and $\Delta t = 0.03s$

## Our contributions

1. [J. Suja, P. Kulmon and M. Benko, *"Scheduling of multi-function multistatic sensor,"* in IEEE Transactions on Aerospace and Electronic Systems, 2025, doi: 10.1109/TAES.2025.3572871](https://ieeexplore.ieee.org/document/11012724)
2. [P. Kulmon, J. Suja and M. Benko, *"Scheduling of Multi-Function Sensor,"* in IEEE Transactions on Radar Systems, vol. 1, pp. 729-739, 2023, doi: 10.1109/TRS.2023.3335208](https://ieeexplore.ieee.org/document/10325557)
3. [J. Pikman, *Optimization of Tuning Plans for a Passive Surveillance System*, diploma thesis at CTU in Prague, under supervision of P. Sucha, P. Kulmon and J.Suja](https://dspace.cvut.cz/handle/10467/114901?locale-attribute=en)
4. [contributions to the NATO STO SET-302](https://www.sto.nato.int/Pages/activitieslisting.aspx)
5. [J. Suja and P. Kulmon, *"Scalarization of Multi-Function Sensor Scheduling Problem,"* 2024 New Trends in Signal Processing (NTSP), Demanovska Dolina, Slovakia, 2024, pp. 1-5, doi: 10.23919/NTSP61680.2024.10726299](https://ieeexplore.ieee.org/document/10726299)

