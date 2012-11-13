using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using VVVV.PluginInterfaces.V2;
using Microsoft.Kinect;
using VVVV.Utils.VMath;

namespace VVVV.Nodes
{
	[PluginInfo(Name = "Skeleton", Category = "Kinect", Version = "Microsoft, Split", Author = "elliotwoods")]
	public class SkeletonSplit : IPluginEvaluate
	{
		[Input("Skeleton")]
		ISpread<Skeleton> FInSkeleton;

		[Output("Tracking ID")]
		ISpread<int> FOutTrackingID;

		[Output("Tracking State")]
		ISpread<SkeletonTrackingState> FOutTrackingState;

		[Output("Position")]
		ISpread<Vector3D> FOutPosition;

		[Output("Joint Type")]
		ISpread<ISpread<JointType>> FOutJointType;

		[Output("Joint Position")]
		ISpread<ISpread<Vector3D>> FOutJointPosition;

		[Output("Joint Tracking State")]
		ISpread<ISpread<JointTrackingState>> FOutJointTrackingState;


		public void Evaluate(int SpreadMax)
		{
			FOutTrackingID.SliceCount = SpreadMax;
			FOutTrackingState.SliceCount = SpreadMax;
			FOutPosition.SliceCount = SpreadMax;

			FOutJointPosition.SliceCount = SpreadMax;
			FOutJointType.SliceCount = SpreadMax;
			FOutJointTrackingState.SliceCount = SpreadMax;

			for (int i = 0; i < SpreadMax; i++)
			{
				if (FInSkeleton[i] == null)
					continue;

				FOutTrackingState[i] = FInSkeleton[i].TrackingState;

				{
					var position = FInSkeleton[i].Position;
					FOutPosition[i] = new Vector3D(position.X, position.Y, position.Z);
				}

				FOutTrackingID[i] = FInSkeleton[i].TrackingId;

				var JointCount = FInSkeleton[i].Joints.Count;
				FOutJointPosition[i].SliceCount = 0;
				FOutJointType[i].SliceCount = 0;
				FOutJointTrackingState[i].SliceCount = 0;
				foreach (Joint joint in FInSkeleton[i].Joints)
				{
					var position = joint.Position;
					FOutJointPosition[i].Add(new Vector3D(position.X, position.Y, position.Z));
					FOutJointTrackingState[i].Add(joint.TrackingState);
					FOutJointType[i].Add(joint.JointType);
				}
			}
		}
	}
}
