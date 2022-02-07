

class Status:
    def __init__(self, name, init_value):
        self.name = name
        self.value = init_value
        self.history = [init_value]

    def store_keep_val(self):
        self.update_val(self.value)

    def update_val(self, value):
        self.value = value
        self.history.append(value)

    def get_val(self):
        return self.value

    def get_history(self):
        return self.history

class StatusList:
    def __init__(self, list):
        self.list = list
    
    def get_status(self, status_name):
        status = [status for status in self.list  if status.name == status_name][0]
        return status.value

    def extract_status(self, status_name):
        status = [status for status in self.list  if status.name == status_name][0]
        return status

class OutputStatusList(StatusList):
    def __init__(self, list):
        super(OutputStatusList, self).__init__(list)

    def update_status(self, status_name, val):
        status = self.extract_status(status_name)
        status.update_val(val)