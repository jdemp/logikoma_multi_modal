from InputMapper import InputMapper


class SpeechMapper(InputMapper):
    def __init__(self,use_static, use_dynamic, use_ml):
        super(SpeechMapper, self).__init__(1,use_static,use_dynamic, use_ml)
        self.static_mapping = {}

    def set_static_mapping(self, mapping_file):
        file = open(mapping_file, 'r')
        lines = file.readlines()
        for line in lines:
            s = line.split('->')
            keys = s[0].split('==')
            action = s[1].strip()
            for k in keys:
                self.static_mapping[k.strip()]=action

    def process(self, input):
        if self.static_mapping.has_key(input.strip()):
            return self.static_mapping[input.strip()]
        else:
            return 'No Action'
