# Exploring vestibulo-ocular adaptation in a closed-loop neuro-robotic experiment using STDP. A simulation study

## Description

This repository contain the source code of the vestibulo-ocular reflex (VOR) experiment described in the IROS paper: "Exploring vestibulo-ocular adaptation in a closed-loop neuro-robotic experiment using STDP. A simulation study". This experiment has been implemented in the neurorobotic platform (NRP) developed in the framework of the Human Brain Project (HBP). This is a simulation platform specially designed to perform brain-body-environment experiments.
In this work we have implemented a spiking neural model of a cerebellum including two plasticity mechanisms able to adapt the cerebellar structure. This cerebellar model is able to compensate the head movement of an iCub robot generating the corresponding eye motor commands.

## Dependencies 

* This experiment has been implemented in the NRP (https://neurorobotics.net/). This platform can be used online or can be installed in local (from source code or docker).
* The cerebellar module including all the neuron model and learning rules must we installed in NEST (https://github.com/jgarridoalcazar/SpikingCerebellum).

## How to configure the experiment in the NRP

* Open sesion in the NRP.
* Clone a template form "tamplates" window.
* Go to "Experimental files" window and delete all the file from the last folder created. Upload in this folder all the files included in this repository.
* The experiment will be available in the "My experiments" window. Selec the "iCub VOR UGR" experiment and push the botton launch.

## Acknowledgment

This research was supported by the “contrato puente” UGR fellowship (F. Naveros), the Juan de la Cierva Spanish fellowship (N. R. Luque), a grant from the European Union (Cerebsensing, H2020-653019 to J. A. Garrido), the European Union ER (HBP-SGA1, H2020-RIA. 720270; HBP-SGA2, H2020-RIA. 785907) and the National Grant (TIN2016-81041-R partially funded by FEDER). 

