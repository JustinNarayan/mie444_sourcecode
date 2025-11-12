class EncoderReading:
    """Represents three encoder distances in inches."""
    __slots__ = ("is_for_localization", "encoder1", "encoder2", "encoder3")

    def __init__(self, is_for_localization=False, encoder1=0.0, encoder2=0.0, encoder3=0.0):
        self.is_for_localization = is_for_localization
        self.encoder1 = encoder1
        self.encoder2 = encoder2
        self.encoder3 = encoder3
        
    def get_readings(self):
        return self.encoder1, self.encoder2, self.encoder3

    def update_from_msg(self, msg):
        """Update the encoder reading from a tuple of a bool and three floats."""
        values = msg.decode()
        if len(values) != 4:
            raise ValueError("Expected a bool three float values for encoder reading")
        self.is_for_localization, self.encoder1, self.encoder2, self.encoder3 = values

    def copy(self):
        """Return a copy of this encoder reading."""
        return EncoderReading(self.is_for_localization, self.encoder1, self.encoder2, self.encoder3)

    def __repr__(self):
        return f"EncoderReading(is_for_loc={self.is_for_localization}, encoder1={self.encoder1:.2f}, encoder2={self.encoder2:.2f}, encoder3={self.encoder3:.2f})"
