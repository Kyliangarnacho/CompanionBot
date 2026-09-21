"""Tiny raw-signal Conv1D multi-task baseline for Stage 4B."""

from __future__ import annotations

import torch
from torch import nn


class TinyTemporalCNN(nn.Module):
    def __init__(self, input_channels: int) -> None:
        super().__init__()
        self.features = nn.Sequential(
            nn.Conv1d(input_channels, 16, kernel_size=7, padding=3),
            nn.BatchNorm1d(16),
            nn.ReLU(),
            nn.Conv1d(16, 32, kernel_size=5, padding=2),
            nn.BatchNorm1d(32),
            nn.ReLU(),
            nn.AdaptiveAvgPool1d(1),
        )
        self.alpha = nn.Linear(32, 1)
        self.slip = nn.Linear(32, 1)
        self.rough = nn.Linear(32, 1)

    def encode(self, x: torch.Tensor) -> torch.Tensor:
        return self.features(x).squeeze(-1)

    def forward_events(self, x: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        latent = self.encode(x)
        return self.slip(latent).squeeze(-1), self.rough(latent).squeeze(-1)

    def forward(self, x: torch.Tensor) -> tuple[torch.Tensor, ...]:
        latent = self.encode(x)
        return (
            self.alpha(latent).squeeze(-1),
            self.slip(latent).squeeze(-1),
            self.rough(latent).squeeze(-1),
        )


class BoundedSlopeResidual(nn.Module):
    """Small correction head; authority is fixed independently of test data."""

    def __init__(self, input_features: int, maximum_correction_deg: float = 3.0) -> None:
        super().__init__()
        self.maximum_correction_deg = float(maximum_correction_deg)
        self.network = nn.Sequential(
            nn.Linear(input_features, 32),
            nn.ReLU(),
            nn.Linear(32, 1),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.maximum_correction_deg * torch.tanh(
            self.network(x).squeeze(-1)
        )
