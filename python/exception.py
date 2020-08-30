class InvalidConfigurationException(Exception):
    '''
    Invalid robot arm configuration exception.
    '''
    pass


class IKError(Exception):
    '''
    Inverse kinematic calculation exception.
    '''
    pass
