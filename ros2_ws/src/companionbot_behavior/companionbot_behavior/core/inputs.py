"""Additive ASR transport metadata, independent of recognition confidence.

captured_at_s remains the existing host audio-callback receipt, not ADC capture
or ASR completion. An adapter must map foreign clocks before Session admission.
"""
from typing import Literal
from pydantic import BaseModel, ConfigDict, Field, model_validator


class ASRMetadata(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True, allow_inf_nan=False)
    contract_version: Literal[2] = 2
    source_id: str = Field(min_length=1)
    source_epoch: str = Field(min_length=1)
    sequence: int = Field(ge=0, strict=True)
    received_at_s: float
    clock_domain: str = "host_perf_counter"
    time_semantics: str = "host_callback_receipt"
    produced_at_s: float | None = None
    produced_clock_domain: str | None = None

    @model_validator(mode="after")
    def production_clock(self):
        if (self.produced_at_s is None) != (self.produced_clock_domain is None):
            raise ValueError("production_time_requires_clock")
        return self
