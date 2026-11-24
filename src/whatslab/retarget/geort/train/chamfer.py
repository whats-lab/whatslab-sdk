import torch


def chamfer_distance(input_points: torch.Tensor, target_points: torch.Tensor) -> torch.Tensor:
    dist_matrix = torch.sum((input_points.unsqueeze(2) - target_points.unsqueeze(1)) ** 2, dim=-1)
    min_dist_a, _ = torch.min(dist_matrix, dim=2)
    min_dist_b, _ = torch.min(dist_matrix, dim=1)
    return (torch.mean(min_dist_a, dim=1) + torch.mean(min_dist_b, dim=1)).mean()
