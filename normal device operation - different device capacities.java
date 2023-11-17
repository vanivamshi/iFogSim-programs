package org.fog.test.perfeval;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.LinkedList;
import java.util.List;
import org.cloudbus.cloudsim.Host;
import org.cloudbus.cloudsim.Log;
import org.cloudbus.cloudsim.Pe;
import org.cloudbus.cloudsim.Storage;
import org.cloudbus.cloudsim.core.CloudSim;
import org.cloudbus.cloudsim.power.PowerHost;
import org.cloudbus.cloudsim.provisioners.RamProvisionerSimple;
import org.cloudbus.cloudsim.sdn.overbooking.BwProvisionerOverbooking;
import org.cloudbus.cloudsim.sdn.overbooking.PeProvisionerOverbooking;
import org.fog.application.AppEdge;
import org.fog.application.AppLoop;
import org.fog.application.Application;
import org.fog.application.selectivity.FractionalSelectivity;
import org.fog.entities.Actuator;
import org.fog.entities.FogBroker;
import org.fog.entities.FogDevice;
import org.fog.entities.FogDeviceCharacteristics;
import org.fog.entities.Sensor;
import org.fog.entities.Tuple;
import org.fog.placement.Controller;
import org.fog.placement.ModuleMapping;
import org.fog.placement.ModulePlacementEdgewards;
import org.fog.placement.ModulePlacementMapping;
import org.fog.policy.AppModuleAllocationPolicy;
import org.fog.scheduler.StreamOperatorScheduler;
import org.fog.utils.FogLinearPowerModel;
import org.fog.utils.FogUtils;
import org.fog.utils.TimeKeeper;
import org.fog.utils.distribution.DeterministicDistribution;
public class TwoApps {
	static List<FogDevice> fogDevices1 = new ArrayList<FogDevice>();
	static List<FogDevice> fogDevices2 = new ArrayList<FogDevice>();
	static List<Sensor> sensors = new ArrayList<Sensor>();
	static List<Actuator> actuators = new ArrayList<Actuator>();
	static int numOfAreas = 2;
	static int numOfCamerasPerArea1=5;
	static double CAM_TRANSMISSION_TIME = 5;
	private static boolean CLOUD = false;
	public static void main(String[] args) {
		Log.printLine("Starting smart car parking system...");
		try {
			Log.disable();
			int num_user = 1; // number of cloud users
			Calendar calendar = Calendar.getInstance();
			boolean trace_flag = false; // mean trace events
			CloudSim.init(num_user, calendar, trace_flag);
			
			String appId1 = "dcns1"; // identifier of the application
			String appId2 = "dcns2"; // identifier of the application
			
			FogBroker broker1 = new FogBroker("broker1");
			FogBroker broker2 = new FogBroker("broker2");

			Application application1 = createApplication1(appId1, broker1.getId());
			Application application2 = createApplication2(appId2, broker2.getId());

			application1.setUserId(broker1.getId());
			application2.setUserId(broker2.getId());
			
			createFogDevices(broker1.getId(), appId1);
			createFogDevices(broker2.getId(), appId2);
			
			Controller controller = null;
			ModuleMapping moduleMapping1 = ModuleMapping.createModuleMapping(); // initializing a module mapping
			ModuleMapping moduleMapping2 = ModuleMapping.createModuleMapping(); // initializing a module mapping
			
			for(FogDevice device : fogDevices1){
				if(device.getName().startsWith("c")){ // names of all Smart Cameras start with 'm' 
					moduleMapping1.addModuleToDevice("picture-capture1", device.getName());  // fixing 1 instance of the Motion Detector module to each Smart Camera
				}
			}
			for(FogDevice device : fogDevices2){
				if(device.getName().startsWith("c")){ // names of all Smart Cameras start with 'm' 
					moduleMapping2.addModuleToDevice("picture-capture2", device.getName());  // fixing 1 instance of the Motion Detector module to each Smart Camera
				}
			}

			for(FogDevice device : fogDevices1){
				if(device.getName().startsWith("a")){ // names of all fog devices start with 'a' 
					moduleMapping1.addModuleToDevice("slot-detector1", device.getName());  // fixing 1 instance of the Motion Detector module to each Smart Camera
				}
			}
			for(FogDevice device : fogDevices2){
				if(device.getName().startsWith("a")){ // names of all fog devices start with 'a' 
					moduleMapping2.addModuleToDevice("slot-detector2", device.getName());  // fixing 1 instance of the Motion Detector module to each Smart Camera
				}
			}

			//moduleMapping.addModuleToDevice("user_interface", "cloud"); // fixing instances of User Interface module in the Cloud
			if(CLOUD){
				// if the mode of deployment is cloud-based
				moduleMapping1.addModuleToDevice("picture-capture1", "cloud"); // placing all instances of Object Detector module in the Cloud
				moduleMapping1.addModuleToDevice("slot-detector1", "cloud"); // placing all instances of Object Tracker module in the Cloud
				moduleMapping2.addModuleToDevice("picture-capture2", "cloud"); // placing all instances of Object Detector module in the Cloud
				moduleMapping2.addModuleToDevice("slot-detector2", "cloud"); // placing all instances of Object Tracker module in the Cloud
			}
			
			controller = new Controller("master-controller", fogDevices1, sensors, 
					actuators);
			controller = new Controller("master-controller", fogDevices2, sensors, 
					actuators);

			
			controller.submitApplication(application1, 
					(CLOUD)?(new ModulePlacementMapping(fogDevices1, application1, moduleMapping1))
							:(new ModulePlacementEdgewards(fogDevices1, sensors, actuators, application1, moduleMapping1)));

			controller.submitApplication(application2, 
					(CLOUD)?(new ModulePlacementMapping(fogDevices2, application2, moduleMapping2))
							:(new ModulePlacementEdgewards(fogDevices2, sensors, actuators, application2, moduleMapping2)));

			TimeKeeper.getInstance().setSimulationStartTime(Calendar.getInstance().getTimeInMillis());
			
			CloudSim.startSimulation();

			CloudSim.stopSimulation();

			Log.printLine("VRGame finished!");
		} catch (Exception e) {
			e.printStackTrace();
			Log.printLine("Unwanted errors happen");
		}
	}
	
	/**
	 * Creates the fog devices in the physical topology of the simulation.
	 * @param userId
	 * @param appId
	 */
	private static void createFogDevices(int userId, String appId) {
		FogDevice cloud = createFogDevice("cloud", 44800, 40000, 100, 10000, 0, 0.01, 16*103, 16*83.25);
		cloud.setParentId(-1);
		fogDevices1.add(cloud);
		fogDevices2.add(cloud);
		FogDevice proxy = createFogDevice("proxy-server", 2800, 4000, 10000, 10000, 1, 0.0, 107.339, 83.4333);
		proxy.setParentId(cloud.getId());
		proxy.setUplinkLatency(100); // latency of connection between proxy server and cloud is 100 ms
		fogDevices1.add(proxy);
		fogDevices2.add(proxy);
		for(int i=0;i<numOfAreas;i++){
			addArea(i+"", userId, appId, proxy.getId());
		}
	}

	private static FogDevice addArea(String id, int userId, String appId, int parentId){
		FogDevice router = createFogDevice("a-"+id, 2800, 10000, 180, 180, 2, 0.0, 1.0, 0.000036);
		fogDevices1.add(router);
		fogDevices2.add(router);
		router.setUplinkLatency(2); // latency of connection between router and proxy server is 2 ms
		for(int i=0;i<numOfCamerasPerArea1;i++){
			String mobileId = id+"-"+i;
			FogDevice camera1 = addCamera1(mobileId, userId, appId, router.getId()); // adding a smart camera to the physical topology. Smart cameras have been modeled as fog devices as well.
			FogDevice camera2 = addCamera2(mobileId, userId, appId, router.getId()); // adding a smart camera to the physical topology. Smart cameras have been modeled as fog devices as well.
			camera1.setUplinkLatency(2); // latency of connection between camera and router is 2 ms
			camera2.setUplinkLatency(2); // latency of connection between camera and router is 2 ms
			fogDevices1.add(camera1);
			fogDevices2.add(camera2);
		}
		router.setParentId(parentId);
		return router;
	}
	
	private static FogDevice addCamera1(String id, int userId, String appId, int parentId){
		FogDevice camera1 = createFogDevice("m-"+id, 500, 10000, 180, 180, 3, 0, 1.0, 0.000036);
		camera1.setParentId(parentId);
		Sensor sensor = new Sensor("s-"+id, "CAMERA", userId, appId, new DeterministicDistribution(5)); // inter-transmission time of camera (sensor) follows a deterministic distribution
		sensors.add(sensor);
		Actuator ptz = new Actuator("ptz-"+id, userId, appId, "PTZ_CONTROL");
		actuators.add(ptz);
		sensor.setGatewayDeviceId(camera1.getId());
		sensor.setLatency(1.0);  // latency of connection between camera (sensor) and the parent Smart Camera is 1 ms
		ptz.setGatewayDeviceId(camera1.getId());
		ptz.setLatency(1.0);  // latency of connection between PTZ Control and the parent Smart Camera is 1 ms
		return camera1;
	}

	private static FogDevice addCamera2(String id, int userId, String appId, int parentId){
		FogDevice camera2 = createFogDevice("m-"+id, 500, 1000, 180, 180, 3, 0, 1.0, 0.000036);
		camera2.setParentId(parentId);
		Sensor sensor = new Sensor("s-"+id, "CAMERA", userId, appId, new DeterministicDistribution(5)); // inter-transmission time of camera (sensor) follows a deterministic distribution
		sensors.add(sensor);
		Actuator ptz = new Actuator("ptz-"+id, userId, appId, "PTZ_CONTROL");
		actuators.add(ptz);
		sensor.setGatewayDeviceId(camera2.getId());
		sensor.setLatency(1.0);  // latency of connection between camera (sensor) and the parent Smart Camera is 1 ms
		ptz.setGatewayDeviceId(camera2.getId());
		ptz.setLatency(1.0);  // latency of connection between PTZ Control and the parent Smart Camera is 1 ms
		return camera2;
	}

	/**
	 * Creates a vanilla fog device
	 * @param nodeName name of the device to be used in simulation
	 * @param mips MIPS
	 * @param ram RAM
	 * @param upBw uplink bandwidth
	 * @param downBw downlink bandwidth
	 * @param level hierarchy level of the device
	 * @param ratePerMips cost rate per MIPS used
	 * @param busyPower
	 * @param idlePower
	 * @return
	 */
	private static FogDevice createFogDevice(String nodeName, long mips,
			int ram, long upBw, long downBw, int level, double ratePerMips, double busyPower, double idlePower) {
		
		List<Pe> peList = new ArrayList<Pe>();

		// 3. Create PEs and add these into a list.
		peList.add(new Pe(0, new PeProvisionerOverbooking(mips))); // need to store Pe id and MIPS Rating

		int hostId = FogUtils.generateEntityId();
		long storage = 1000; // host storage
		int bw = 180;

		PowerHost host = new PowerHost(
				hostId,
				new RamProvisionerSimple(ram),
				new BwProvisionerOverbooking(bw),
				storage,
				peList,
				new StreamOperatorScheduler(peList),
				new FogLinearPowerModel(busyPower, idlePower)
			);
		List<Host> hostList = new ArrayList<Host>();
		hostList.add(host);
		String arch = "x86"; // system architecture
		String os = "Linux"; // operating system
		String vmm = "Xen";
		double time_zone = 10.0; // time zone this resource located
		double cost = 3.0; // the cost of using processing in this resource
		double costPerMem = 0.05; // the cost of using memory in this resource
		double costPerStorage = 0.001; // the cost of using storage in this
										// resource
		double costPerBw = 0.0; // the cost of using bw in this resource
		LinkedList<Storage> storageList = new LinkedList<Storage>(); // we are not adding SAN
													// devices by now
		FogDeviceCharacteristics characteristics = new FogDeviceCharacteristics(
				arch, os, vmm, host, time_zone, cost, costPerMem,
				costPerStorage, costPerBw);
		FogDevice fogdevice = null;
		try {
			fogdevice = new FogDevice(nodeName, characteristics, 
					new AppModuleAllocationPolicy(hostList), storageList, 10, upBw, downBw, 0, ratePerMips);
		} catch (Exception e) {
			e.printStackTrace();
		}
		fogdevice.setLevel(level);
		return fogdevice;
	}

	/**
	 * Function to create the Intelligent Surveillance application in the DDF model. 
	 * @param appId unique identifier of the application
	 * @param userId identifier of the user of the application
	 * @return
	 */
	@SuppressWarnings({"serial" })
	private static Application createApplication1(String appId, int userId){
		
		Application application = Application.createApplication(appId, userId);
		/*
		 * Adding modules (vertices) to the application model (directed graph)
		 */
		application.addAppModule("picture-capture1", 10);
		application.addAppModule("slot-detector1", 10);
        application.addAppModule("processingModule", 10); // adding module Concentration Calculator to the application model
        application.addAppModule("storageModule", 10); // adding module Connector to the application model
		
		/*
		 * Connecting the application modules (vertices) in the application model (directed graph) with edges
		 */
		application.addAppEdge("CAMERA1", "picture-capture1", 1000, 500, "CAMERA1", Tuple.UP, AppEdge.SENSOR); // adding edge from CAMERA (sensor) to Motion Detector module carrying tuples of type CAMERA
		application.addAppEdge("picture-capture1", "processingModule", 1000, 500, "processor",Tuple.UP, AppEdge.MODULE);
		application.addAppEdge("processingModule", "storageModule", 1000, 500, "storage",Tuple.UP, AppEdge.MODULE);
		application.addAppEdge("storageModule", "slot-detector1", 1000, 500, "slots1",Tuple.DOWN, AppEdge.MODULE);
		// adding edge from Slot Detector to PTZ CONTROL (actuator)
		application.addAppEdge("slot-detector1", "PTZ_CONTROL1", 100, 28, 100, "PTZ_PARAMS1", Tuple.DOWN, AppEdge.ACTUATOR);
		
		application.addTupleMapping("picture-capture1", "CAMERA1", "CAMERA1", new FractionalSelectivity(1.0));
		application.addTupleMapping("processingModule", "picture-capture1", "processor", new FractionalSelectivity(1.0));
		application.addTupleMapping("storageModule", "processingModule", "storage", new FractionalSelectivity(1.0));
		application.addTupleMapping("slot-detector1", "storageModule", "slots1", new FractionalSelectivity(1.0));
		application.addTupleMapping("PTZ_CONTROL1", "slot-detector1", "PTZ_PARAMS1", new FractionalSelectivity(1.0));
		
		final AppLoop loop1 = new AppLoop(new ArrayList<String>() {{
			add("CAMERA1");
			add("picture-capture1");
			add("processingModule");
			add("storageModule");
			add("slot-detector1");
			add("PTZ_CONTROL1");
		}});
		
		List<AppLoop> loops = new ArrayList<AppLoop>() {{
			add(loop1);
		}};
		
		application.setLoops(loops);
		return application;
		}

	@SuppressWarnings({"serial" })
	private static Application createApplication2(String appId, int userId){
		
		Application application = Application.createApplication(appId, userId);
		/*
		 * Adding modules (vertices) to the application model (directed graph)
		 */
		application.addAppModule("picture-capture2", 10);
		application.addAppModule("slot-detector2", 10);
        application.addAppModule("processingModule", 10); // adding module Concentration Calculator to the application model
        application.addAppModule("storageModule", 10); // adding module Connector to the application model
		
		/*
		 * Connecting the application modules (vertices) in the application model (directed graph) with edges
		 */
		application.addAppEdge("CAMERA2", "picture-capture2", 1000, 500, "CAMERA2", Tuple.UP, AppEdge.SENSOR); // adding edge from CAMERA (sensor) to Motion Detector module carrying tuples of type CAMERA
		application.addAppEdge("picture-capture2", "processingModule", 1000, 500, "processor",Tuple.UP, AppEdge.MODULE);
		application.addAppEdge("processingModule", "storageModule", 1000, 500, "storage",Tuple.UP, AppEdge.MODULE);
		application.addAppEdge("storageModule", "slot-detector2", 1000, 500, "slots2",Tuple.DOWN, AppEdge.MODULE);
		// adding edge from Slot Detector to PTZ CONTROL (actuator)
		application.addAppEdge("slot-detector2", "PTZ_CONTROL2", 100, 28, 100, "PTZ_PARAMS2", Tuple.DOWN, AppEdge.ACTUATOR);
		
		application.addTupleMapping("picture-capture2", "CAMERA2", "CAMERA2", new FractionalSelectivity(1.0));
		application.addTupleMapping("processingModule", "picture-capture2", "processor", new FractionalSelectivity(1.0));
		application.addTupleMapping("storageModule", "processingModule", "storage", new FractionalSelectivity(1.0));
		application.addTupleMapping("slot-detector2", "storageModule", "slots2", new FractionalSelectivity(1.0));
		application.addTupleMapping("PTZ_CONTROL2", "slot-detector2", "PTZ_PARAMS2", new FractionalSelectivity(1.0));
		
		final AppLoop loop2 = new AppLoop(new ArrayList<String>() {{
			add("CAMERA2");
			add("picture-capture2");
			add("processingModule");
			add("storageModule");
			add("slot-detector2");
			add("PTZ_CONTROL2");
		}});
		
		List<AppLoop> loops = new ArrayList<AppLoop>() {{
			add(loop2);
		}};
		
		application.setLoops(loops);
		return application;
		}
	}