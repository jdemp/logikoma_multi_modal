INPUT_TYPES = {0:'Keyboard', 1:'Voice/Speech', 2:'Gesture', 3:'Other'}


class InputMapper(object):
    def __init__(self, input_type,use_static, use_dynamic, use_ml):
        self.input_type = input_type
        self.maps_in_use = {'static':use_static,'dynamic':use_dynamic,'ml':use_ml}

    def __str__(self):
        return INPUT_TYPES[self.input_type]

    def process(self, input):
        pass
