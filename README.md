# Exploring vestibulo-ocular adaptation in a closed-loop neuro-robotic experiment using STDP. A simulation study

## Description

This repository contains the source code of the vestibulo-ocular reflex (VOR) experiment described in the IROS paper: "Exploring vestibulo-ocular adaptation in a closed-loop neuro-robotic experiment using STDP. A simulation study". This experiment has been implemented in the neurorobotic platform (NRP) developed in the framework of the Human Brain Project (HBP). This is a simulation platform specially designed to perform brain-body-environment experiments.
In this work we have implemented a spiking neural model of a cerebellum including two plasticity mechanisms able to adapt the cerebellar structure. This cerebellar model is able to compensate the head movement of an iCub robot generating the corresponding eye motor commands.

## Dependencies 

* This experiment has been implemented in the NRP (https://neurorobotics.net/). This platform can be installed in local using docker(http://www.neurorobotics.net/local_install.html).
* The cerebellar module including all the neuron model and learning rules is installed in NEST (https://github.com/jgarridoalcazar/SpikingCerebellum).

## How to configure the experiment in the NRP

* Install the NRP using the nrp_installer script (http://www.neurorobotics.net/local_install.html)
* Upgrade the pyNN version in the backend:
./nrp_installer connect_backend
pip install --upgrade pyNN
* Launch the NRP
./nrp_installer start
Open the url "http://localhost:9000/#/esv-private". user: nrpuser; password:password
* Create a new experiment and upload all the files in this folder.
* Launch the iCub_VOR_UGR experiment.

## Acknowledgment

This research was supported by the “contrato puente” UGR fellowship (F. Naveros), the Juan de la Cierva Spanish fellowship (N. R. Luque), a grant from the European Union (Cerebsensing, H2020-653019 to J. A. Garrido), the European Union ER (HBP-SGA1, H2020-RIA. 720270; HBP-SGA2, H2020-RIA. 785907) and the National Grant (TIN2016-81041-R partially funded by FEDER). 

