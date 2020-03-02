# Overview

Fenix tiller pilot is a totally self-contained autopilot designed for tiller steered sailboats. The autopilot is mounted between the tiller and a single attachment point on the boat’s structure. It is designed for owner installation and is ready for use after connection to the boat’s 12 V electrical system.

## Operating modes

The tiller pilot has three basic operating modes:

* Standby mode: autopilot off
* Auto mode: autopilot engaged and locked onto a heading
* Track mode: autopilot on and maintaining a track between two waypoints created on a navigation system

## Tiller pilot control

Fenix tiller pilot can implement different devices to control the autopilot. 

{% hint style="info" %}
At least one Control Device shall be available with minimum User Functions implemented. For the purpose of this guide, Virtuino Mobile App will be considered.
{% endhint %}

### Virtuino Mobile App

An App based on Virtuino is available to Android OS mobiles.

This App provides full capability to control tiller pilot and limited configuration capabilities.

### $PEMC Serial Interface

Fenix Autopilot implements a complete set of $PEMC messages to commission and configure the system from a PC/laptop.

* $PEMC are messages following NMEA 0183 rules.
* $PEMC messages also allows additional fixed and hand-held Fenix Autopilot compatible control units to be easily connected at alternative steering and control positions.

### NMEA Interface

The tiller pilot is NMEA compatible, so it can share data transmitted from other NMEA instruments, track information, from any navigator transmiting NMEA 0183 data \(eg. OpenCPN\), enables the autopilot to provide waypoint control.

## About this Handbook

This handbook contains important information about installing, using and maintaining Fenix tiller pilot. To get the best from the system, please read this handbook thoroughly.

## Safety notices

#### WARNING: Product installation 

{% hint style="danger" %}
WARNING: Passage making under autopilot control can lead to the relaxation of the permanent watch. Always maintain a permanent watch no matter how clear the sea may appear to be.
{% endhint %}

#### WARNING: Electrical safety 

{% hint style="danger" %}
Make sure the power supply is switched off before you make any electrical connections.
{% endhint %}

#### WARNING: Navigation aid 

{% hint style="danger" %}
Although we have designed this product to be accurate and reliable, many factors can affect its performance. As a result, it should only be used as an aid to navigation and should never replace common sense and navigational judgement. Always maintain a permanent watch so you can respond to situations as they develop.
{% endhint %}

It is the skipper’s responsibility to ensure the safety of the boat at all times by following these basic rules: 

* [x] Ensure that someone is present at the helm AT ALL TIMES, to take manual control in an emergency.
* [x] Make sure that all crew members know how to disengage the autopilot.
* [x] Regularly check for other boats and any obstacles to navigation – no matter how clear the sea may appear, a dangerous situation can develop rapidly.
* [x] Maintain an accurate record of the boat’s position by using either a navigation aid or visual bearings.
* [x] Maintain a continuous plot of your boat’s position on a current chart. Ensure that the locked autopilot heading will steer the boat clear of all obstacles. Make proper allowance for tidal set – the autopilot cannot.
* [x] Even when your autopilot is locked onto the desired track using a navigation aid, always maintain a log and make regular positional plots. Navigation signals can produce significant errors under some circumstances and the autopilot will not be able to detect these errors.

