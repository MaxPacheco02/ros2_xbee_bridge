import re

import yaml
from digi.xbee.devices import RemoteDigiMeshDevice
from digi.xbee.models.address import XBee64BitAddress


class XbeeNeigbors(object):
    def __init__(self, namespace, local, config_file) -> None:
        with open(config_file, 'r') as f:
            self.xbee_devices = yaml.safe_load(f)
        
        self.address_to_name = {xd[1].upper(): xd[0] for xd in self.xbee_devices if xd[0] != namespace}
        self.id_to_name = {xd[2]: xd[0] for xd in self.xbee_devices if xd[0] != namespace}
        if local is not None:
            self.name_to_object = {xd[0]: RemoteDigiMeshDevice(local, XBee64BitAddress.from_hex_string(xd[1]), xd[2]) 
                                   for xd in self.xbee_devices if xd[0] != namespace}
        
    def __getitem__(self, name):
        return self.name_to_object[name]
        
    def get_name(self, lookup):
        mac_pattern = re.compile(r'^([0-9A-Fa-f]{2}){8}$')
        if mac_pattern.match(lookup):
            return self.address_to_name[lookup.upper()]
        else:
            return self.id_to_name[lookup]
        
    @property
    def namespaces(self):
        return set(self.id_to_name.values())
        