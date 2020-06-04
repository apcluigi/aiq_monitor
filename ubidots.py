# -*- coding: utf-8 -*-
# @Author: lorenzo
# @Date:   2017-09-21 16:17:16
# @Last Modified by:   m.cipriani
# @Last Modified time: 2020-01-30 17:04:43

"""
.. module:: iot
***************
Ubidots Library
***************
The Zerynth Ubidots Library can be used to ease the connection to the `Ubidots IoT platform <https://ubidots.com/>`_.
It allows to make your device act as an Ubidots Device which can be created through Ubidots dashboard.
    """

import json
import ssl
from mqtt import mqtt


class UbiMQTTClient(mqtt.Client):

    def __init__(self, mqtt_id, user_type, api_token, ssl_ctx):
        mqtt.Client.__init__(self, mqtt_id, clean_session=True)
        mqtt.Client.set_username_pw(self, api_token)
        endpoints = {'educational': 'things.ubidots.com', 'business': 'industrial.api.ubidots.com'}
        self.endpoint = endpoints[user_type]
        self.ssl_ctx = ssl_ctx

    def connect(self, port=8883):
        if port == 1883:
            mqtt.Client.connect(self, self.endpoint, 60, port=port)
        else:
            mqtt.Client.connect(self, self.endpoint, 60, port=port, ssl_ctx=self.ssl_ctx)


class Device:
    """
===============
The Device class
===============
.. class:: Device(device_label, user_type, api_token)
        Create a Device instance representing an Ubidots Device.
        The Device object will contain an mqtt client instance pointing to Ubidots MQTT broker located at :samp:`things.ubidots.com` or :samp:`industrial.api.ubidots.com` depending on :samp:`user_type`, a string which can be :samp:`"educational"` or :samp:`"business"`.
        The client is configured with :samp:`device_label` as MQTT id and is able to connect securely through TLS and to authenticate setting :samp:`api_token` as client username.
        The client is accessible through :samp:`mqtt` instance attribute and exposes all :ref:`Zerynth MQTT Client methods <lib.zerynth.mqtt>` so that it is possible, for example, to setup
        custom callback on MQTT commands (though the Device class already exposes high-level methods to setup Ubidots specific callbacks).
        The only difference concerns mqtt.connect method which does not require broker url and ssl context, taking them from Device configuration::
            my_device = iot.Device('my_label', 'business', 'my_api_token')
            my_device.mqtt.connect()
            ...
            my_device.mqtt.loop()
    """

    def __init__(self, device_label, user_type, api_token):
        self.ctx = ssl.create_ssl_context(options=ssl.CERT_NONE) # TODO: add root cert
        self.mqtt = UbiMQTTClient(device_label, user_type, api_token, self.ctx)
        self.device_label = device_label
        self._variables_cbks = {}

    def publish(self, data, variable=None):
        """
.. method:: publish(data, variable=None)
        Publish :samp:`data` dictionary to device or device variable :samp:`variable`.
        Data dictionary should follow `valid Ubidots data format <https://ubidots.com/docs/api/mqtt.html#publish>`_.
        """
        if variable is None:
            self.mqtt.publish('/v1.6/devices/' + self.device_label, json.dumps(data))
        else:
            self.mqtt.publish('/v1.6/devices/' + self.device_label + '/' + variable, json.dumps(data))

    def _is_variable_update(self, mqtt_data):
        if ('message' in mqtt_data):
            return mqtt_data['message'].topic.startswith('/v1.6/devices/') and not mqtt_data['message'].retain
        return False

    def _handle_variable_update(self, mqtt_client, mqtt_data):
        tt = mqtt_data['message'].topic.split('/')
        if tt[-1] == 'lv':
            variable = tt[-2]
            device = tt[-3]
            value = float(mqtt_data['message'].payload)
        else:
            variable = tt[-1]
            device = tt[-2]
            value = json.loads(mqtt_data['message'].payload)

        self._variables_cbks[device][variable](value)

    def on_variable_update(self, device, variable, callback, json=True):
        """
.. method:: on_variable_update(device, variable, callback, json=True)
        Set a callback to respond to :samp:`variable` updates from device :samp:`device`.
        :samp:`callback` will be called passing a dictionary or a float value, containing variable updates, respectively for a :samp:`True` or :samp:`False` :samp:`json` parameter value::
            def noise_callback(data):
                noise_level = data['value']
                noise_location = data['context']
                print(noise_level, noise_location)
            device.on_variable_update('noise-listener', 'noise-level', noise_callback)
        """
        if not device in self._variables_cbks or not variable in self._variables_cbks[device]:
            if json:
                self.mqtt.subscribe([['/v1.6/devices/' + device + '/' + variable, 0]])
            else:
                self.mqtt.subscribe([['/v1.6/devices/' + device + '/' + variable + '/lv', 0]])
            if not device in self._variables_cbks:
                self._variables_cbks[device] = {}
        self._variables_cbks[device][variable] = callback
        self.mqtt.on(mqtt.PUBLISH, self._handle_variable_update, self._is_variable_update)