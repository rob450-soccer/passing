import json
import numpy as np
import torch
from torch_policy import TorchPolicyGRU, load_policy_from_files

def export_gru_policy_to_onnx(weights_path, meta_path, output_file):
    device = torch.device("cpu")
    model, meta = load_policy_from_files(weights_path, meta_path, device)
    model.eval()

    # Infer obs dim from the first linear layer's input
    obs_dim = model.gru_obs_encoder_dense.in_features  # policy_obs_dim after index selection
    # But we need the FULL obs dim, which we get from policy_observation_indices
    full_obs_dim = int(model.policy_observation_indices.max().item()) + 1
    gru_hidden_dim = model.gru_hidden_dim

    dummy_obs = torch.randn(1, full_obs_dim)
    dummy_carry = torch.zeros(1, gru_hidden_dim)

    torch.onnx.export(
        model,
        (dummy_obs, dummy_carry),
        output_file,
        input_names=["obs", "carry_in"],
        output_names=["action", "carry_out"],
        dynamic_axes={
            "obs":       {0: "batch"},
            "carry_in":  {0: "batch"},
            "carry_out": {0: "batch"},
        },
        opset_version=17,
    )
    print(f"Exported to {output_file} (obs_dim={full_obs_dim}, gru_hidden_dim={gru_hidden_dim})")

    # Save dims alongside so walk.py can load them
    with open(output_file + ".meta.json", "w") as f:
        json.dump({"full_obs_dim": full_obs_dim, "gru_hidden_dim": gru_hidden_dim}, f)


export_gru_policy_to_onnx("locomotion_nn.pth", "locomotion_nn_meta.json", "new_nn_walk.onnx")